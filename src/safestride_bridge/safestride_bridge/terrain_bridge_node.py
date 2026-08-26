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
from sensor_msgs.msg import Imu, Range

try:
    import serial
except ImportError:  # pragma: no cover - target package dependency
    serial = None

from .protocol import (
    BOARD_ROLE_TERRAIN,
    FIRMWARE_RELEASE_ID,
    PROTOCOL_SCHEMA_ID,
    PROTOCOL_VERSION,
    Frame,
    FrameParser,
    HelloPayload,
    PacketType,
    PayloadDecodeError,
    SessionStartPayload,
    TerrainTelemetryPayload,
    sequence_is_newer,
)
from .validation import bounded_int, finite_float


CAP_TOF10120 = 1 << 8
CAP_BNO055 = 1 << 9


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
        self._compatibility_error: Optional[str] = None
        self._session_id = 0
        self._boot_id = 0
        self._capabilities = 0
        self._session_started = False
        self._tx_sequence = 0
        self._last_sequence: Optional[int] = None
        self._last_telemetry_time: Optional[float] = None
        self._last_telemetry: Optional[TerrainTelemetryPayload] = None

        self._tof_pub = self.create_publisher(
            Range, self._topic_tof, qos_profile_sensor_data
        )
        self._imu_pub = self.create_publisher(
            Imu, self._topic_imu, qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            TerrainStatus, self._topic_status, 10
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
            ('diagnostics.publish_rate_hz', 1.0),
            ('range.min_m', 0.10),
            ('range.max_m', 2.00),
            ('range.field_of_view_rad', 0.052),
            ('frames.tof', 'terrain_tof_link'),
            ('frames.bno', 'imu_link'),
            ('topics.tof', '/terrain/tof'),
            ('topics.imu', '/terrain/imu'),
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
        self._frame_bno = str(self._value('frames.bno'))
        self._topic_tof = str(self._value('topics.tof'))
        self._topic_imu = str(self._value('topics.imu'))
        self._topic_status = str(self._value('topics.status'))
        self._topic_diagnostics = str(self._value('topics.diagnostics'))

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
        self._compatibility_error = None
        self._session_started = False
        self._last_sequence = None
        self._last_telemetry_time = None
        self._last_telemetry = None

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
        if frame.packet_type != PacketType.TERRAIN_TELEMETRY:
            self._payload_errors += 1
            return
        if not self._session_started or frame.session_id != self._session_id:
            self._session_errors += 1
            return
        try:
            telemetry = TerrainTelemetryPayload.unpack(frame.payload)
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
        self._last_telemetry_time = now
        self._last_telemetry = telemetry
        self._publish_telemetry(telemetry)

    def _handle_hello(self, hello: HelloPayload) -> None:
        compatibility_errors = []
        if hello.board_role != BOARD_ROLE_TERRAIN:
            compatibility_errors.append(
                f'board role {hello.board_role} is not TERRAIN'
            )
        if hello.protocol_version != PROTOCOL_VERSION:
            compatibility_errors.append(
                f'HELLO protocol {hello.protocol_version} != '
                f'{PROTOCOL_VERSION}'
            )
        if hello.schema_id != PROTOCOL_SCHEMA_ID:
            compatibility_errors.append(
                f'schema 0x{hello.schema_id:04x} != '
                f'0x{PROTOCOL_SCHEMA_ID:04x}'
            )
        if hello.firmware_release_id != FIRMWARE_RELEASE_ID:
            compatibility_errors.append(
                f'firmware release {hello.firmware_release_id} != '
                f'{FIRMWARE_RELEASE_ID}'
            )
        if compatibility_errors:
            self._compatibility_error = '; '.join(compatibility_errors)
            self._session_started = False
            return
        self._compatibility_error = None
        self._parser.last_unsupported_version = None

        if (
            hello.boot_id == self._boot_id
            and self._session_started
            and self._link_ok()
        ):
            # Terrain firmware advertises periodically for reconnects. A
            # healthy session must not be restarted for each advertisement.
            return
        # Reaching this point means there is no healthy session. Always issue
        # a fresh session ID, even if the MCU boot ID did not change.
        new_session = True
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
        frame = Frame(
            packet_type=PacketType.SESSION_START,
            sequence=self._next_sequence(),
            session_id=self._session_id,
            timestamp_ms=int(self._now() * 1000.0) & 0xFFFFFFFF,
            payload=SessionStartPayload(
                hello.boot_id,
                BOARD_ROLE_TERRAIN,
            ).pack(),
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
        self._publish_imu(telemetry)
        self._publish_status()

    @staticmethod
    def _quaternion_from_rpy(
        roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _publish_imu(self, telemetry: TerrainTelemetryPayload) -> None:
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_bno
        message.angular_velocity_covariance[0] = -1.0
        message.linear_acceleration_covariance[0] = -1.0
        if telemetry.bno_valid:
            yaw = telemetry.bno_heading_mrad / 1000.0
            roll = telemetry.bno_roll_mrad / 1000.0
            pitch = telemetry.bno_pitch_mrad / 1000.0
            (
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ) = self._quaternion_from_rpy(roll, pitch, yaw)
            message.orientation_covariance[0] = 0.05
            message.orientation_covariance[4] = 0.05
            message.orientation_covariance[8] = 0.05
        else:
            message.orientation.w = 1.0
            message.orientation_covariance[0] = -1.0
        self._imu_pub.publish(message)

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
            message.yaw_rad = float('nan')
            message.pitch_rad = float('nan')
            message.roll_rad = float('nan')
            message.bno_valid = False
            message.bno_calibration = 0
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
            message.bno_valid = bool(telemetry.bno_valid)
            message.bno_calibration = telemetry.bno_calibration
            message.yaw_rad = (
                telemetry.bno_heading_mrad / 1000.0
                if telemetry.bno_valid else float('nan')
            )
            message.pitch_rad = (
                telemetry.bno_pitch_mrad / 1000.0
                if telemetry.bno_valid else float('nan')
            )
            message.roll_rad = (
                telemetry.bno_roll_mrad / 1000.0
                if telemetry.bno_valid else float('nan')
            )
        message.mpu_valid = False
        message.leg_state = int(
            getattr(TerrainStatus, 'LEG_SAFE_STOP', 4)
        )
        message.retracted_limit = False
        message.deployed_limit = False
        self._status_pub.publish(message)

    def _diagnostic_tick(self) -> None:
        self._publish_status()
        now = self._now()
        telemetry = self._last_telemetry
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
        elif self._parser.last_unsupported_version is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                'protocol version mismatch: Terrain Uno '
                f'v{self._parser.last_unsupported_version}, '
                f'bridge v{PROTOCOL_VERSION}; flash both MCUs'
            )
        elif self._compatibility_error is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                f'incompatible Terrain Uno: {self._compatibility_error}'
            )
        elif not self._session_started:
            status.level = DiagnosticStatus.WARN
            status.message = 'waiting for Terrain Uno HELLO'
        elif not self._link_ok(now):
            status.level = DiagnosticStatus.ERROR
            status.message = 'terrain telemetry timed out'
        elif not (self._capabilities & CAP_TOF10120):
            status.level = DiagnosticStatus.ERROR
            status.message = 'Terrain firmware lacks TOF-10120 capability'
        elif not (self._capabilities & CAP_BNO055):
            status.level = DiagnosticStatus.ERROR
            status.message = 'Terrain firmware lacks BNO055 capability'
        elif telemetry is None or not telemetry.tof_valid:
            status.level = DiagnosticStatus.ERROR
            status.message = 'TOF-10120 reading invalid'
        elif not telemetry.bno_valid:
            status.level = DiagnosticStatus.ERROR
            status.message = 'BNO055 reading invalid'
        elif (
            self._parser.crc_error_count
            or self._parser.frame_error_count
            or self._payload_errors
            or self._session_errors
            or self._sequence_errors
        ):
            status.level = DiagnosticStatus.WARN
            status.message = 'terrain link active with malformed frames'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'terrain link, TOF-10120 and BNO055 active'
        age = (
            float('inf') if self._last_telemetry_time is None
            else max(0.0, now - self._last_telemetry_time)
        )
        status.values = [
            KeyValue(key='port', value=self._port),
            KeyValue(key='expected_board_role', value='TERRAIN'),
            KeyValue(key='protocol_version', value=str(PROTOCOL_VERSION)),
            KeyValue(
                key='protocol_schema_id',
                value=f'0x{PROTOCOL_SCHEMA_ID:04x}',
            ),
            KeyValue(
                key='firmware_release_id',
                value=str(FIRMWARE_RELEASE_ID),
            ),
            KeyValue(
                key='observed_protocol_version',
                value=(
                    str(self._parser.last_unsupported_version)
                    if self._parser.last_unsupported_version is not None
                    else (
                        str(PROTOCOL_VERSION)
                        if self._boot_id else 'unknown'
                    )
                ),
            ),
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
                key='bno_valid',
                value=str(bool(telemetry and telemetry.bno_valid)).lower(),
            ),
            KeyValue(
                key='bno_calibration',
                value=(
                    f'0x{telemetry.bno_calibration:02x}'
                    if telemetry is not None else 'unknown'
                ),
            ),
            KeyValue(
                key='crc_errors', value=str(self._parser.crc_error_count)
            ),
            KeyValue(
                key='version_errors',
                value=str(self._parser.version_error_count),
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
