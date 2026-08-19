"""ROS 2 serial bridge for the SafeStride terrain sensor controller."""

from __future__ import annotations

import math
import secrets
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from safestride_interfaces.msg import TerrainStatus
from sensor_msgs.msg import NavSatFix, NavSatStatus, Range
from std_msgs.msg import Float32

try:
    import serial
except ImportError:  # pragma: no cover - target package dependency
    serial = None

from .protocol import (
    Frame,
    FrameParser,
    GpsTelemetryPayload,
    HelloPayload,
    PacketType,
    PayloadDecodeError,
    SessionStartPayload,
    TerrainTelemetryPayload,
    sequence_is_newer,
)
from .validation import bounded_int, finite_float


CAP_TOF10120 = 1 << 8
CAP_GPS_NMEA = 1 << 9


class TerrainBridgeNode(Node):
    """Convert Terrain Uno COBS/CRC telemetry into ROS sensor topics."""

    def __init__(self) -> None:
        super().__init__('terrain_bridge')
        self._declare_parameters()
        self._load_parameters()

        self._serial = None
        self._last_open_attempt = float('-inf')
        self._parser = FrameParser(self._max_frame_size)
        self._payload_errors = 0
        self._session_errors = 0
        self._sequence_errors = 0
        self._session_id = 0
        self._boot_id = 0
        self._capabilities = 0
        self._session_started = False
        self._tx_sequence = 0
        self._controller_capability_error: Optional[str] = None
        self._last_sequence: Optional[int] = None
        self._last_telemetry_time: Optional[float] = None
        self._last_telemetry: Optional[TerrainTelemetryPayload] = None
        self._last_gps_time: Optional[float] = None
        self._last_gps: Optional[GpsTelemetryPayload] = None

        self._tof_pub = self.create_publisher(
            Range, self._topic_tof, qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            TerrainStatus, self._topic_status, 10
        )
        self._gps_fix_pub = self.create_publisher(
            NavSatFix, self._topic_gps_fix, 10
        )
        self._gps_speed_pub = self.create_publisher(
            Float32, self._topic_gps_speed, 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._topic_diagnostics, 10
        )
        self._io_timer = self.create_timer(
            1.0 / self._poll_rate_hz, self._io_tick
        )
        self._diagnostic_timer = self.create_timer(
            1.0 / self._diagnostic_rate_hz, self._diagnostic_tick
        )
        self.get_logger().info(
            f'configured terrain bridge for {self._port} at '
            f'{self._baudrate} baud'
        )

    def _declare_parameters(self) -> None:
        parameters = (
            ('serial.port', '/dev/safestride-terrain'),
            ('serial.baudrate', 115200),
            ('serial.read_chunk_size', 256),
            ('serial.max_frame_size', 160),
            ('serial.reconnect_period_s', 1.0),
            ('transport.poll_rate_hz', 100.0),
            ('telemetry.timeout_s', 0.30),
            ('gps.enabled', True),
            ('gps.timeout_s', 2.00),
            ('diagnostics.publish_rate_hz', 1.0),
            ('range.min_m', 0.10),
            ('range.max_m', 2.00),
            ('range.field_of_view_rad', 0.052),
            ('frames.tof', 'terrain_tof_link'),
            ('frames.gps', 'gps_link'),
            ('topics.tof', '/terrain/tof'),
            ('topics.gps_fix', '/gps/fix'),
            ('topics.gps_speed', '/gps/speed'),
            ('topics.status', '/terrain/status'),
            ('topics.diagnostics', '/diagnostics'),
        )
        for name, default in parameters:
            self.declare_parameter(name, default)

    def _value(self, name: str):
        return self.get_parameter(name).value

    def _load_parameters(self) -> None:
        self._port = str(self._value('serial.port'))
        if not self._port.strip():
            raise ValueError('serial.port must not be empty')
        self._baudrate = bounded_int(
            'serial.baudrate', self._value('serial.baudrate'),
            minimum=1200, maximum=2000000,
        )
        self._read_chunk_size = bounded_int(
            'serial.read_chunk_size', self._value('serial.read_chunk_size'),
            minimum=1, maximum=4096,
        )
        self._max_frame_size = bounded_int(
            'serial.max_frame_size', self._value('serial.max_frame_size'),
            minimum=64, maximum=4096,
        )
        self._reconnect_period = finite_float(
            'serial.reconnect_period_s',
            self._value('serial.reconnect_period_s'),
            minimum=0.0, maximum=60.0, minimum_inclusive=False,
        )
        self._poll_rate_hz = finite_float(
            'transport.poll_rate_hz', self._value('transport.poll_rate_hz'),
            minimum=0.0, maximum=2000.0, minimum_inclusive=False,
        )
        self._telemetry_timeout = finite_float(
            'telemetry.timeout_s', self._value('telemetry.timeout_s'),
            minimum=0.0, maximum=5.0, minimum_inclusive=False,
        )
        self._gps_enabled = bool(self._value('gps.enabled'))
        self._gps_timeout = finite_float(
            'gps.timeout_s', self._value('gps.timeout_s'),
            minimum=0.0, maximum=30.0, minimum_inclusive=False,
        )
        self._diagnostic_rate_hz = finite_float(
            'diagnostics.publish_rate_hz',
            self._value('diagnostics.publish_rate_hz'),
            minimum=0.0, maximum=100.0, minimum_inclusive=False,
        )
        self._range_min = finite_float(
            'range.min_m', self._value('range.min_m'),
            minimum=0.0, maximum=1000.0,
        )
        self._range_max = finite_float(
            'range.max_m', self._value('range.max_m'),
            minimum=0.0, maximum=1000.0, minimum_inclusive=False,
        )
        self._range_fov = finite_float(
            'range.field_of_view_rad', self._value('range.field_of_view_rad'),
            minimum=0.0, maximum=2.0 * math.pi,
            minimum_inclusive=False,
        )
        if self._range_max <= self._range_min:
            raise ValueError('range.max_m must exceed range.min_m')
        if 1.0 / self._poll_rate_hz >= self._telemetry_timeout:
            raise ValueError(
                'serial poll period must be shorter than telemetry timeout'
            )
        self._frame_tof = str(self._value('frames.tof'))
        self._frame_gps = str(self._value('frames.gps'))
        self._topic_tof = str(self._value('topics.tof'))
        self._topic_gps_fix = str(self._value('topics.gps_fix'))
        self._topic_gps_speed = str(self._value('topics.gps_speed'))
        self._topic_status = str(self._value('topics.status'))
        self._topic_diagnostics = str(self._value('topics.diagnostics'))
        required_names = (
            self._frame_tof,
            self._frame_gps,
            self._topic_tof,
            self._topic_gps_fix,
            self._topic_gps_speed,
        )
        if any(not value.strip() for value in required_names):
            raise ValueError('terrain frame and topic names must not be empty')

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _connected(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

    def _link_ok(self, now: Optional[float] = None) -> bool:
        if (
            not self._connected()
            or not self._session_started
            or self._last_telemetry_time is None
        ):
            return False
        if now is None:
            now = self._now()
        return now - self._last_telemetry_time <= self._telemetry_timeout

    def _reset_link(self) -> None:
        self._parser.reset()
        self._session_id = 0
        self._boot_id = 0
        self._capabilities = 0
        self._session_started = False
        self._controller_capability_error = None
        self._last_sequence = None
        self._last_telemetry_time = None
        self._last_telemetry = None
        self._last_gps_time = None
        self._last_gps = None

    def _close_serial(self, reason: str) -> None:
        port = self._serial
        self._serial = None
        self._reset_link()
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        self.get_logger().warning(f'terrain serial link closed: {reason}')

    def _try_open_serial(self, now: float) -> None:
        if (
            serial is None
            or now - self._last_open_attempt < self._reconnect_period
        ):
            return
        self._last_open_attempt = now
        try:
            port = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0,
                write_timeout=0.05,
            )
            port.reset_input_buffer()
            port.reset_output_buffer()
        except (OSError, serial.SerialException) as error:
            self.get_logger().warning(
                f'cannot open {self._port}: {error}',
                throttle_duration_sec=5.0,
            )
            return
        self._serial = port
        self._reset_link()
        self.get_logger().info(
            f'opened {self._port}; waiting for Terrain Uno HELLO'
        )

    def _io_tick(self) -> None:
        now = self._now()
        if not self._connected():
            self._try_open_serial(now)
            return
        try:
            waiting = int(self._serial.in_waiting)
            if waiting <= 0:
                return
            data = self._serial.read(min(waiting, self._read_chunk_size))
        except (OSError, serial.SerialException) as error:
            self._close_serial(str(error))
            return
        for frame in self._parser.feed(data):
            self._handle_frame(frame, now)

    def _handle_frame(self, frame: Frame, now: float) -> None:
        if frame.packet_type == PacketType.HELLO:
            if frame.session_id != 0:
                self._session_errors += 1
                return
            try:
                hello = HelloPayload.unpack(frame.payload)
            except PayloadDecodeError:
                self._payload_errors += 1
                return
            self._handle_hello(hello)
            return
        if frame.packet_type not in (
            PacketType.TERRAIN_TELEMETRY,
            PacketType.GPS_TELEMETRY,
        ):
            self._payload_errors += 1
            return
        if not self._session_started or frame.session_id != self._session_id:
            self._session_errors += 1
            return
        try:
            if frame.packet_type == PacketType.TERRAIN_TELEMETRY:
                payload = TerrainTelemetryPayload.unpack(frame.payload)
            else:
                payload = GpsTelemetryPayload.unpack(frame.payload)
        except PayloadDecodeError:
            self._payload_errors += 1
            return
        if (
            self._last_sequence is not None
            and not sequence_is_newer(frame.sequence, self._last_sequence)
        ):
            self._sequence_errors += 1
            return
        self._last_sequence = frame.sequence
        if isinstance(payload, TerrainTelemetryPayload):
            self._last_telemetry_time = now
            self._last_telemetry = payload
            self._publish_telemetry(payload)
        else:
            self._last_gps_time = now
            self._last_gps = payload
            self._publish_gps(payload)

    def _handle_hello(self, hello: HelloPayload) -> None:
        missing_capabilities = []
        if not (hello.capabilities & CAP_TOF10120):
            missing_capabilities.append('TOF-10120')
        if self._gps_enabled and not (hello.capabilities & CAP_GPS_NMEA):
            missing_capabilities.append('GPS NMEA')
        if missing_capabilities:
            error = (
                'device on the terrain port is not Terrain firmware '
                f'({", ".join(missing_capabilities)} capability missing)'
            )
            self._session_id = 0
            self._boot_id = hello.boot_id
            self._capabilities = hello.capabilities
            self._session_started = False
            self._controller_capability_error = error
            self._last_sequence = None
            self._last_telemetry_time = None
            self._last_telemetry = None
            self._last_gps_time = None
            self._last_gps = None
            self.get_logger().error(
                error,
                throttle_duration_sec=5.0,
            )
            return
        self._controller_capability_error = None
        new_session = (
            not self._session_started or hello.boot_id != self._boot_id
        )
        if new_session:
            session_id = secrets.randbits(32)
            if session_id == 0 or session_id == self._session_id:
                session_id = (self._session_id + 1) & 0xFFFFFFFF or 1
            self._session_id = session_id
            self._boot_id = hello.boot_id
            self._capabilities = hello.capabilities
            self._last_sequence = None
            self._last_telemetry_time = None
            self._last_telemetry = None
            self._last_gps_time = None
            self._last_gps = None
        frame = Frame(
            packet_type=PacketType.SESSION_START,
            sequence=self._next_sequence(),
            session_id=self._session_id,
            timestamp_ms=int(self._now() * 1000.0) & 0xFFFFFFFF,
            payload=SessionStartPayload(hello.boot_id).pack(),
        )
        if self._write_frame(frame):
            self._session_started = True
            if new_session:
                self.get_logger().info(
                    f'started terrain session 0x{self._session_id:08x} '
                    f'for boot 0x{self._boot_id:08x}'
                )

    def _next_sequence(self) -> int:
        sequence = self._tx_sequence
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFF
        return sequence

    def _write_frame(self, frame: Frame) -> bool:
        if not self._connected():
            return False
        try:
            encoded = frame.encode()
            written = self._serial.write(encoded)
            if written != len(encoded):
                raise serial.SerialTimeoutException(
                    f'partial serial write ({written}/{len(encoded)} bytes)'
                )
            return True
        except (
            OSError,
            serial.SerialException,
            serial.SerialTimeoutException,
        ) as error:
            self._close_serial(str(error))
            return False

    def _publish_telemetry(
        self, telemetry: TerrainTelemetryPayload
    ) -> None:
        message = Range()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_tof
        message.radiation_type = Range.INFRARED
        message.field_of_view = self._range_fov
        message.min_range = self._range_min
        message.max_range = self._range_max
        message.range = (
            telemetry.tof_distance_mm / 1000.0
            if telemetry.tof_valid
            else float('nan')
        )
        self._tof_pub.publish(message)
        self._publish_status()

    def _publish_status(self) -> None:
        now = self._now()
        telemetry = self._last_telemetry
        message = TerrainStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_tof
        if telemetry is None:
            message.tof_distance_m = float('nan')
            message.tof_valid = False
            message.fault_bits = 0
            message.telemetry_age = float('inf')
        else:
            message.tof_distance_m = (
                telemetry.tof_distance_mm / 1000.0
                if telemetry.tof_valid
                else float('nan')
            )
            message.tof_valid = bool(telemetry.tof_valid)
            message.fault_bits = telemetry.fault_bits
            message.telemetry_age = float(
                max(0.0, now - self._last_telemetry_time)
            )
        message.pitch_rad = float('nan')
        message.roll_rad = float('nan')
        message.mpu_valid = False
        message.bno_valid = False
        message.bno_calibration = 0
        message.leg_state = int(
            getattr(TerrainStatus, 'LEG_SAFE_STOP', 4)
        )
        message.retracted_limit = False
        message.deployed_limit = False
        self._status_pub.publish(message)

    def _publish_gps(self, gps: GpsTelemetryPayload) -> None:
        stamp = self.get_clock().now().to_msg()
        fix_valid = bool(gps.flags & gps.FLAG_FIX_VALID)
        speed_valid = bool(gps.flags & gps.FLAG_SPEED_VALID)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self._frame_gps
        fix.status.status = (
            NavSatStatus.STATUS_FIX
            if fix_valid else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = gps.latitude_e7 / 10000000.0 if fix_valid else math.nan
        fix.longitude = (
            gps.longitude_e7 / 10000000.0 if fix_valid else math.nan
        )
        fix.altitude = math.nan
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._gps_fix_pub.publish(fix)

        speed = Float32()
        speed.data = gps.speed_mm_s / 1000.0 if speed_valid else math.nan
        self._gps_speed_pub.publish(speed)

    def _diagnostic_tick(self) -> None:
        self._publish_status()
        now = self._now()
        telemetry = self._last_telemetry
        gps = self._last_gps
        status = DiagnosticStatus()
        status.name = 'SafeStride terrain serial bridge'
        status.hardware_id = (
            f'{self._port}/boot-{self._boot_id:08x}'
            if self._boot_id else self._port
        )
        if serial is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'pyserial is not installed'
        elif not self._connected():
            status.level = DiagnosticStatus.ERROR
            status.message = 'terrain serial port disconnected'
        elif self._controller_capability_error is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = self._controller_capability_error
        elif not self._session_started:
            status.level = DiagnosticStatus.WARN
            status.message = 'waiting for Terrain Uno HELLO'
        elif not self._link_ok(now):
            status.level = DiagnosticStatus.ERROR
            status.message = 'terrain telemetry timed out'
        elif telemetry is None or not telemetry.tof_valid:
            status.level = DiagnosticStatus.ERROR
            status.message = 'TOF-10120 reading invalid'
        elif (
            self._parser.crc_error_count
            or self._parser.frame_error_count
            or self._payload_errors
            or self._session_errors
            or self._sequence_errors
        ):
            status.level = DiagnosticStatus.WARN
            status.message = 'terrain link active with malformed frames'
        elif self._gps_enabled and (
            self._last_gps_time is None
            or now - self._last_gps_time > self._gps_timeout
        ):
            status.level = DiagnosticStatus.WARN
            status.message = 'terrain active; GPS telemetry is stale'
        elif self._gps_enabled and (
            gps is None
            or not (gps.flags & gps.FLAG_FIX_VALID)
            or not (gps.flags & gps.FLAG_SPEED_VALID)
        ):
            status.level = DiagnosticStatus.WARN
            status.message = 'terrain active; GPS is waiting for a fix'
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                'terrain, TOF-10120 and GPS active'
                if self._gps_enabled
                else 'terrain link and TOF-10120 active'
            )
        age = (
            float('inf') if self._last_telemetry_time is None
            else max(0.0, now - self._last_telemetry_time)
        )
        status.values = [
            KeyValue(key='port', value=self._port),
            KeyValue(key='boot_id', value=f'0x{self._boot_id:08x}'),
            KeyValue(key='session_id', value=f'0x{self._session_id:08x}'),
            KeyValue(key='telemetry_age_s', value=f'{age:.3f}'),
            KeyValue(
                key='tof_valid',
                value=str(bool(telemetry and telemetry.tof_valid)).lower(),
            ),
            KeyValue(
                key='tof_distance_mm',
                value=(
                    str(telemetry.tof_distance_mm)
                    if telemetry is not None else 'unknown'
                ),
            ),
            KeyValue(
                key='gps_fix_valid',
                value=str(bool(
                    gps and gps.flags & gps.FLAG_FIX_VALID
                )).lower(),
            ),
            KeyValue(
                key='gps_speed_valid',
                value=str(bool(
                    gps and gps.flags & gps.FLAG_SPEED_VALID
                )).lower(),
            ),
            KeyValue(
                key='gps_satellites',
                value=str(gps.satellites if gps is not None else 0),
            ),
            KeyValue(
                key='gps_speed_mps',
                value=(
                    f'{gps.speed_mm_s / 1000.0:.3f}'
                    if gps is not None
                    and gps.flags & gps.FLAG_SPEED_VALID else 'nan'
                ),
            ),
            KeyValue(
                key='crc_errors', value=str(self._parser.crc_error_count)
            ),
            KeyValue(
                key='frame_errors',
                value=str(
                    self._parser.frame_error_count + self._payload_errors
                    + self._session_errors + self._sequence_errors
                ),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
