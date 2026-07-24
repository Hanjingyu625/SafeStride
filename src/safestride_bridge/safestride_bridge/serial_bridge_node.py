"""ROS 2 node that bridges SafeStride topics to the serial wire protocol."""

from __future__ import annotations

import math
import secrets
import threading
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from safestride_interfaces.msg import WalkerStatus
from sensor_msgs.msg import BatteryState, JointState, Range
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:  # pragma: no cover - depends on target OS packaging
    serial = None

from .protocol import (
    CommandPayload,
    Frame,
    FrameParser,
    HelloPayload,
    PacketType,
    PayloadDecodeError,
    SessionStartPayload,
    TelemetryPayload,
    sequence_is_newer,
)
from .validation import bounded_int, finite_float


# Firmware status flags.
STATUS_SESSION = 1 << 0
STATUS_MOTOR_ENABLED = 1 << 1
STATUS_DEADMAN = 1 << 2
STATUS_ESTOP = 1 << 3
STATUS_WATCHDOG_TIMEOUT = 1 << 4
STATUS_COMMAND_SEEN = 1 << 5
STATUS_STATE_SHIFT = 8
STATUS_STATE_MASK = 0x7

# Firmware state values encoded in status_bits[10:8].
FW_BOOT = 0
FW_DISARMED = 1
FW_ARMED = 2
FW_SAFE_STOP = 3
FW_ESTOP = 4
FW_FAULT = 5


def _int32_delta(current: int, previous: int) -> int:
    """Return wrap-safe signed delta between two int32 counters."""

    delta = (int(current) - int(previous)) & 0xFFFFFFFF
    if delta & 0x80000000:
        delta -= 0x100000000
    return delta


def _quaternion_z(yaw: float) -> tuple[float, float]:
    """Return the z and w components of a yaw-only quaternion."""

    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class SerialBridgeNode(Node):
    """Own the serial port, protocol session and ROS-facing state."""

    def __init__(self) -> None:
        super().__init__('serial_bridge')
        self._declare_parameters()
        self._load_parameters()

        self._lock = threading.RLock()
        self._serial = None
        self._last_open_attempt = float('-inf')
        self._parser = FrameParser(self._max_frame_size)
        self._payload_error_count = 0
        self._session_error_count = 0
        self._sequence_error_count = 0

        self._session_id = 0
        self._boot_id = 0
        self._capabilities = 0
        self._session_started = False
        self._tx_sequence = 0

        self._enabled_requested = False
        self._last_command_time: Optional[float] = None
        self._target_linear = 0.0
        self._target_angular = 0.0
        self._command_generation = 0
        self._command_timed_out = True
        self._arm_neutral_remaining = 0
        self._arm_confirmed = False
        self._arm_confirmation_deadline: Optional[float] = None
        self._post_arm_neutral_generation = 0
        self._post_arm_neutral_seen = False

        self._last_telemetry_time: Optional[float] = None
        self._last_telemetry: Optional[TelemetryPayload] = None
        self._last_telemetry_sequence: Optional[int] = None
        self._last_encoder_left: Optional[int] = None
        self._last_encoder_right: Optional[int] = None
        self._joint_left = 0.0
        self._joint_right = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        self._joint_pub = self.create_publisher(
            JointState, self._topic_joint_states, 10
        )
        self._odom_pub = self.create_publisher(
            Odometry, self._topic_odom, 10
        )
        self._range_left_pub = self.create_publisher(
            Range, self._topic_range_left, qos_profile_sensor_data
        )
        self._range_right_pub = self.create_publisher(
            Range, self._topic_range_right, qos_profile_sensor_data
        )
        self._battery_pub = self.create_publisher(
            BatteryState, self._topic_battery, qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            WalkerStatus, self._topic_status, 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._topic_diagnostics, 10
        )
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self._publish_tf else None
        )

        self._cmd_sub = self.create_subscription(
            TwistStamped,
            self._topic_cmd_vel,
            self._on_cmd_vel,
            10,
        )
        self._enable_service = self.create_service(
            SetBool,
            self._service_set_enabled,
            self._on_set_enabled,
        )

        self._io_timer = self.create_timer(
            1.0 / self._poll_rate_hz, self._io_tick
        )
        self._command_timer = self.create_timer(
            1.0 / self._command_rate_hz, self._command_tick
        )
        self._diagnostic_timer = self.create_timer(
            1.0 / self._diagnostic_rate_hz, self._diagnostic_tick
        )

        self.get_logger().info(
            f'configured serial bridge for {self._port} at '
            f'{self._baudrate} baud; starts disarmed'
        )

    def _declare_parameters(self) -> None:
        parameters = (
            ('serial.port', '/dev/serial/by-id/CHANGE_ME'),
            ('serial.baudrate', 115200),
            ('serial.read_chunk_size', 256),
            ('serial.max_frame_size', 160),
            ('serial.reconnect_period_s', 1.0),
            ('transport.poll_rate_hz', 200.0),
            ('command.publish_rate_hz', 50.0),
            ('command.timeout_s', 0.20),
            ('command.ttl_ms', 200),
            ('command.arm_neutral_cycles', 5),
            ('command.arm_max_wheel_speed_rad_s', 0.10),
            ('command.arm_confirmation_timeout_s', 1.0),
            ('telemetry.timeout_s', 0.30),
            ('diagnostics.publish_rate_hz', 1.0),
            ('base.wheel_radius_m', 0.15),
            ('base.wheel_separation_m', 0.55),
            ('base.ticks_per_revolution', 1024),
            ('base.max_wheel_speed_rad_s', 3.0),
            ('range.min_m', 0.02),
            ('range.max_m', 4.0),
            ('range.field_of_view_rad', 0.35),
            ('battery.empty_voltage', 0.0),
            ('battery.full_voltage', 0.0),
            ('frames.odom', 'odom'),
            ('frames.base', 'base_footprint'),
            ('frames.range_left', 'front_left_range_link'),
            ('frames.range_right', 'front_right_range_link'),
            ('joints.left', 'left_wheel_joint'),
            ('joints.right', 'right_wheel_joint'),
            ('topics.cmd_vel', '/cmd_vel_safe'),
            ('topics.joint_states', '/joint_states'),
            ('topics.odom', '/odom'),
            ('topics.range_left', '/range/front_left'),
            ('topics.range_right', '/range/front_right'),
            ('topics.battery', '/battery_state'),
            ('topics.status', '/walker/status'),
            ('topics.diagnostics', '/diagnostics'),
            ('services.set_enabled', '/walker/set_enabled'),
            ('publish_tf', True),
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
            'serial.baudrate',
            self._value('serial.baudrate'),
            minimum=1200,
            maximum=2000000,
        )
        self._read_chunk_size = bounded_int(
            'serial.read_chunk_size',
            self._value('serial.read_chunk_size'),
            minimum=1,
            maximum=4096,
        )
        self._max_frame_size = bounded_int(
            'serial.max_frame_size',
            self._value('serial.max_frame_size'),
            minimum=64,
            maximum=4096,
        )
        self._reconnect_period = finite_float(
            'serial.reconnect_period_s',
            self._value('serial.reconnect_period_s'),
            minimum=0.0,
            maximum=60.0,
            minimum_inclusive=False,
        )
        self._poll_rate_hz = finite_float(
            'transport.poll_rate_hz',
            self._value('transport.poll_rate_hz'),
            minimum=0.0,
            maximum=2000.0,
            minimum_inclusive=False,
        )
        self._command_rate_hz = finite_float(
            'command.publish_rate_hz',
            self._value('command.publish_rate_hz'),
            minimum=0.0,
            maximum=500.0,
            minimum_inclusive=False,
        )
        self._command_timeout = finite_float(
            'command.timeout_s',
            self._value('command.timeout_s'),
            minimum=0.0,
            maximum=5.0,
            minimum_inclusive=False,
        )
        self._command_ttl_ms = bounded_int(
            'command.ttl_ms',
            self._value('command.ttl_ms'),
            minimum=20,
            maximum=250,
        )
        self._arm_neutral_cycles = bounded_int(
            'command.arm_neutral_cycles',
            self._value('command.arm_neutral_cycles'),
            minimum=1,
            maximum=100,
        )
        self._arm_max_wheel_speed = finite_float(
            'command.arm_max_wheel_speed_rad_s',
            self._value('command.arm_max_wheel_speed_rad_s'),
            minimum=0.0,
            maximum=100.0,
        )
        self._arm_confirmation_timeout = finite_float(
            'command.arm_confirmation_timeout_s',
            self._value('command.arm_confirmation_timeout_s'),
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._telemetry_timeout = finite_float(
            'telemetry.timeout_s',
            self._value('telemetry.timeout_s'),
            minimum=0.0,
            maximum=5.0,
            minimum_inclusive=False,
        )
        self._diagnostic_rate_hz = finite_float(
            'diagnostics.publish_rate_hz',
            self._value('diagnostics.publish_rate_hz'),
            minimum=0.0,
            maximum=100.0,
            minimum_inclusive=False,
        )

        self._wheel_radius = finite_float(
            'base.wheel_radius_m',
            self._value('base.wheel_radius_m'),
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._wheel_separation = finite_float(
            'base.wheel_separation_m',
            self._value('base.wheel_separation_m'),
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._ticks_per_revolution = finite_float(
            'base.ticks_per_revolution',
            self._value('base.ticks_per_revolution'),
            minimum=0.0,
            maximum=1.0e9,
            minimum_inclusive=False,
        )
        self._max_wheel_speed = finite_float(
            'base.max_wheel_speed_rad_s',
            self._value('base.max_wheel_speed_rad_s'),
            minimum=0.0,
            maximum=2147483.0,
            minimum_inclusive=False,
        )
        self._range_min = finite_float(
            'range.min_m',
            self._value('range.min_m'),
            minimum=0.0,
            maximum=1000.0,
        )
        self._range_max = finite_float(
            'range.max_m',
            self._value('range.max_m'),
            minimum=0.0,
            maximum=1000.0,
            minimum_inclusive=False,
        )
        if self._range_max <= self._range_min:
            raise ValueError('range.max_m must exceed range.min_m')
        self._range_fov = finite_float(
            'range.field_of_view_rad',
            self._value('range.field_of_view_rad'),
            minimum=0.0,
            maximum=2.0 * math.pi,
            minimum_inclusive=False,
        )
        self._battery_empty = finite_float(
            'battery.empty_voltage',
            self._value('battery.empty_voltage'),
            minimum=0.0,
            maximum=1000.0,
        )
        self._battery_full = finite_float(
            'battery.full_voltage',
            self._value('battery.full_voltage'),
            minimum=0.0,
            maximum=1000.0,
        )
        if (
            (self._battery_empty != 0.0 or self._battery_full != 0.0)
            and self._battery_full <= self._battery_empty
        ):
            raise ValueError(
                'battery.full_voltage must exceed battery.empty_voltage'
            )
        if self._arm_max_wheel_speed > self._max_wheel_speed:
            raise ValueError(
                'command.arm_max_wheel_speed_rad_s must not exceed '
                'base.max_wheel_speed_rad_s'
            )
        command_period = 1.0 / self._command_rate_hz
        if command_period >= self._command_timeout:
            raise ValueError(
                'command publish period must be shorter than timeout_s'
            )
        if command_period * 1000.0 >= self._command_ttl_ms:
            raise ValueError(
                'command publish period must be shorter than ttl_ms'
            )
        if 1.0 / self._poll_rate_hz >= self._telemetry_timeout:
            raise ValueError(
                'serial poll period must be shorter than telemetry timeout'
            )

        self._frame_odom = str(self._value('frames.odom'))
        self._frame_base = str(self._value('frames.base'))
        self._frame_range_left = str(self._value('frames.range_left'))
        self._frame_range_right = str(self._value('frames.range_right'))
        self._joint_name_left = str(self._value('joints.left'))
        self._joint_name_right = str(self._value('joints.right'))
        self._topic_cmd_vel = str(self._value('topics.cmd_vel'))
        self._topic_joint_states = str(self._value('topics.joint_states'))
        self._topic_odom = str(self._value('topics.odom'))
        self._topic_range_left = str(self._value('topics.range_left'))
        self._topic_range_right = str(self._value('topics.range_right'))
        self._topic_battery = str(self._value('topics.battery'))
        self._topic_status = str(self._value('topics.status'))
        self._topic_diagnostics = str(self._value('topics.diagnostics'))
        self._service_set_enabled = str(
            self._value('services.set_enabled')
        )
        self._publish_tf = bool(self._value('publish_tf'))

    def _now_monotonic(self) -> float:
        return time.monotonic()

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _serial_connected(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

    def _link_ok(self, now: Optional[float] = None) -> bool:
        if not self._serial_connected() or not self._session_started:
            return False
        if (
            self._last_telemetry_time is None
            or self._last_telemetry is None
            or not (
                self._last_telemetry.status_bits & STATUS_SESSION
            )
        ):
            return False
        if now is None:
            now = self._now_monotonic()
        return (now - self._last_telemetry_time) <= self._telemetry_timeout

    @staticmethod
    def _firmware_state(telemetry: TelemetryPayload) -> int:
        return (
            telemetry.status_bits >> STATUS_STATE_SHIFT
        ) & STATUS_STATE_MASK

    def _firmware_status_consistent(
        self, telemetry: TelemetryPayload
    ) -> bool:
        firmware_state = self._firmware_state(telemetry)
        motor_enabled = bool(
            telemetry.status_bits & STATUS_MOTOR_ENABLED
        )
        return (
            firmware_state in (
                FW_BOOT,
                FW_DISARMED,
                FW_ARMED,
                FW_SAFE_STOP,
                FW_ESTOP,
                FW_FAULT,
            )
            and motor_enabled == (firmware_state == FW_ARMED)
        )

    def _remote_allows_enable(self, telemetry: TelemetryPayload) -> bool:
        firmware_state = self._firmware_state(telemetry)
        return (
            self._firmware_status_consistent(telemetry)
            and bool(telemetry.status_bits & STATUS_SESSION)
            and bool(telemetry.status_bits & STATUS_DEADMAN)
            and not bool(telemetry.status_bits & STATUS_ESTOP)
            and not bool(
                telemetry.status_bits & STATUS_WATCHDOG_TIMEOUT
            )
            and telemetry.fault_bits == 0
            and firmware_state in (FW_DISARMED, FW_ARMED)
        )

    def _clear_enable_request(self) -> None:
        self._enabled_requested = False
        self._arm_neutral_remaining = 0
        self._arm_confirmed = False
        self._arm_confirmation_deadline = None
        self._post_arm_neutral_generation = self._command_generation
        self._post_arm_neutral_seen = False

    def _reset_link_state(self) -> None:
        self._session_id = 0
        self._boot_id = 0
        self._capabilities = 0
        self._session_started = False
        self._clear_enable_request()
        self._last_command_time = None
        self._command_timed_out = True
        self._last_telemetry_time = None
        self._last_telemetry = None
        self._last_telemetry_sequence = None
        self._last_encoder_left = None
        self._last_encoder_right = None
        self._parser.reset()

    def _close_serial(self, reason: str) -> None:
        with self._lock:
            port = self._serial
            self._serial = None
            self._reset_link_state()
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
        self.get_logger().warning(f'serial link closed: {reason}')

    def _try_open_serial(self, now: float) -> None:
        if serial is None:
            return
        if now - self._last_open_attempt < self._reconnect_period:
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

        with self._lock:
            self._serial = port
            self._reset_link_state()
        self.get_logger().info(
            f'opened {self._port}; waiting for controller HELLO'
        )

    def _io_tick(self) -> None:
        now = self._now_monotonic()
        if not self._serial_connected():
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
                self._session_error_count += 1
                return
            try:
                hello = HelloPayload.unpack(frame.payload)
            except PayloadDecodeError:
                self._payload_error_count += 1
                return
            self._handle_hello(hello, frame.sequence)
            return

        if frame.packet_type != PacketType.TELEMETRY:
            self._payload_error_count += 1
            return
        if not self._session_started or frame.session_id != self._session_id:
            self._session_error_count += 1
            return
        try:
            telemetry = TelemetryPayload.unpack(frame.payload)
        except PayloadDecodeError:
            self._payload_error_count += 1
            return
        if (
            self._last_telemetry_sequence is not None
            and not sequence_is_newer(
                frame.sequence, self._last_telemetry_sequence
            )
        ):
            self._sequence_error_count += 1
            return

        self._last_telemetry_sequence = frame.sequence
        self._last_telemetry_time = now
        self._last_telemetry = telemetry
        firmware_state = self._firmware_state(telemetry)
        controller_armed = bool(
            telemetry.status_bits & STATUS_MOTOR_ENABLED
        ) and firmware_state == FW_ARMED
        if (
            self._enabled_requested
            and controller_armed
            and not self._arm_confirmed
        ):
            if (
                self._arm_confirmation_deadline is not None
                and now >= self._arm_confirmation_deadline
            ):
                self._clear_enable_request()
                self._send_command(0, 0, False)
                self.get_logger().warning(
                    'ignored late ARMED confirmation; '
                    'explicit re-enable required'
                )
            else:
                self._arm_confirmed = True
                self._arm_confirmation_deadline = None
                self._post_arm_neutral_generation = (
                    self._command_generation
                )
                self._post_arm_neutral_seen = False
        elif self._arm_confirmed and not controller_armed:
            self._clear_enable_request()
            self.get_logger().warning(
                'controller disarmed unexpectedly; explicit re-enable required'
            )
        self._publish_telemetry(telemetry)

    def _handle_hello(
        self, hello: HelloPayload, hello_sequence: int
    ) -> None:
        if (
            hello.boot_id == self._boot_id
            and self._session_started
            and self._last_telemetry_sequence is not None
            and not sequence_is_newer(
                hello_sequence,
                self._last_telemetry_sequence,
            )
        ):
            # A HELLO buffered before the active session must not tear down a
            # healthy link. A real same-boot session loss sends a newer frame.
            return
        new_session = (
            hello.boot_id != self._boot_id
            or not self._session_started
            or self._last_telemetry_time is not None
        )
        if new_session:
            session_id = secrets.randbits(32)
            if session_id == 0 or session_id == self._session_id:
                session_id = (self._session_id + 1) & 0xFFFFFFFF
                if session_id == 0:
                    session_id = 1
            self._session_id = session_id
            self._boot_id = hello.boot_id
            self._capabilities = hello.capabilities
            self._clear_enable_request()
            self._last_command_time = None
            self._command_timed_out = True
            self._last_telemetry_time = None
            self._last_telemetry = None
            self._last_telemetry_sequence = None
            self._last_encoder_left = None
            self._last_encoder_right = None

        payload = SessionStartPayload(hello.boot_id).pack()
        frame = self._make_frame(
            PacketType.SESSION_START,
            payload,
            session_id=self._session_id,
        )
        if self._write_frame(frame):
            self._session_started = True
            if new_session:
                self.get_logger().info(
                    f'started disarmed session 0x{self._session_id:08x} '
                    f'for boot 0x{self._boot_id:08x}'
                )

    def _next_sequence(self) -> int:
        sequence = self._tx_sequence
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFF
        return sequence

    def _make_frame(
        self,
        packet_type: int,
        payload: bytes,
        session_id: Optional[int] = None,
    ) -> Frame:
        return Frame(
            packet_type=packet_type,
            sequence=self._next_sequence(),
            session_id=(
                self._session_id if session_id is None else session_id
            ),
            timestamp_ms=int(self._now_monotonic() * 1000.0) & 0xFFFFFFFF,
            payload=payload,
        )

    def _write_frame(self, frame: Frame) -> bool:
        if not self._serial_connected():
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

    def _on_cmd_vel(self, message: TwistStamped) -> None:
        linear = float(message.twist.linear.x)
        angular = float(message.twist.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().error('discarded non-finite /cmd_vel_safe')
            return
        with self._lock:
            self._target_linear = linear
            self._target_angular = angular
            self._last_command_time = self._now_monotonic()
            self._command_generation += 1
            self._command_timed_out = False

    def _on_set_enabled(self, request, response):
        now = self._now_monotonic()
        if not request.data:
            with self._lock:
                self._clear_enable_request()
            delivered = self._send_command(0, 0, False)
            response.success = delivered
            if delivered:
                response.message = (
                    'local enable gate closed and disable command sent'
                )
            else:
                response.message = (
                    'local enable gate closed, but the disable command could '
                    'not be delivered; controller watchdog stop is expected'
                )
            return response

        if not self._link_ok(now):
            response.success = False
            response.message = 'cannot enable: controller telemetry is stale'
            return response
        telemetry = self._last_telemetry
        if telemetry is None:
            response.success = False
            response.message = 'cannot enable: no controller telemetry'
            return response
        if not self._firmware_status_consistent(telemetry):
            response.success = False
            response.message = (
                'cannot enable: controller status fields are inconsistent'
            )
            return response
        if telemetry.status_bits & STATUS_ESTOP:
            response.success = False
            response.message = 'cannot enable: emergency stop is active'
            return response
        if telemetry.status_bits & STATUS_WATCHDOG_TIMEOUT:
            response.success = False
            response.message = (
                'cannot enable: controller watchdog reset is pending'
            )
            return response
        firmware_state = self._firmware_state(telemetry)
        if firmware_state == FW_ESTOP:
            response.success = False
            response.message = 'cannot enable: controller is in ESTOP state'
            return response
        if telemetry.fault_bits:
            response.success = False
            response.message = (
                f'cannot enable: controller fault 0x'
                f'{telemetry.fault_bits:04x}'
            )
            return response
        if firmware_state == FW_FAULT:
            response.success = False
            response.message = 'cannot enable: controller is in FAULT state'
            return response
        if firmware_state == FW_SAFE_STOP:
            response.success = False
            response.message = (
                'cannot enable: controller is in SAFE_STOP; '
                'wait for the disabled reset command'
            )
            return response
        if not (telemetry.status_bits & STATUS_SESSION):
            response.success = False
            response.message = 'cannot enable: controller session is inactive'
            return response
        if not (telemetry.status_bits & STATUS_DEADMAN):
            response.success = False
            response.message = 'cannot enable: dead-man switch is not active'
            return response
        if (
            abs(self._target_linear) > 1.0e-6
            or abs(self._target_angular) > 1.0e-6
        ):
            response.success = False
            response.message = (
                'cannot enable: supervised velocity target is not neutral'
            )
            return response
        maximum_measured_speed_mrad_s = int(
            round(self._arm_max_wheel_speed * 1000.0)
        )
        if (
            abs(telemetry.velocity_left_mrad_s)
            > maximum_measured_speed_mrad_s
            or abs(telemetry.velocity_right_mrad_s)
            > maximum_measured_speed_mrad_s
        ):
            response.success = False
            response.message = (
                'cannot enable: powered wheels are not stationary'
            )
            return response
        if (
            self._last_command_time is None
            or now - self._last_command_time > self._command_timeout
        ):
            response.success = False
            response.message = 'cannot enable: no fresh velocity command'
            return response

        with self._lock:
            self._enabled_requested = True
            self._arm_neutral_remaining = self._arm_neutral_cycles
            self._arm_confirmed = False
            self._arm_confirmation_deadline = (
                now + self._arm_confirmation_timeout
            )
            self._post_arm_neutral_generation = self._command_generation
            self._post_arm_neutral_seen = False
        response.success = True
        response.message = (
            'enable gate opened; motion still requires fresh velocity commands'
        )
        return response

    def _command_tick(self) -> None:
        if not self._session_started or not self._serial_connected():
            return
        now = self._now_monotonic()
        link_ok = self._link_ok(now)
        telemetry = self._last_telemetry
        remote_safe = (
            telemetry is not None
            and self._remote_allows_enable(telemetry)
        )
        if not link_ok or not remote_safe:
            self._clear_enable_request()
            self._send_command(0, 0, False)
            return

        fresh = (
            self._last_command_time is not None
            and (now - self._last_command_time) <= self._command_timeout
        )

        if not fresh:
            if not self._command_timed_out:
                self.get_logger().warning(
                    'velocity command timed out; commanding a disabled stop'
                )
            self._command_timed_out = True
            self._clear_enable_request()
            self._send_command(0, 0, False)
            return

        if (
            self._enabled_requested
            and not self._arm_confirmed
            and (
                self._arm_confirmation_deadline is None
                or now >= self._arm_confirmation_deadline
            )
        ):
            self.get_logger().warning(
                'controller did not confirm ARMED before the deadline; '
                'explicit re-enable required'
            )
            self._clear_enable_request()
            self._send_command(0, 0, False)
            return

        if self._enabled_requested and self._arm_neutral_remaining > 0:
            if self._send_command(0, 0, True):
                self._arm_neutral_remaining -= 1
            return
        if self._enabled_requested and not self._arm_confirmed:
            self._send_command(0, 0, True)
            return
        if self._enabled_requested and not self._post_arm_neutral_seen:
            new_command_after_arm = (
                self._command_generation
                > self._post_arm_neutral_generation
            )
            neutral_command = (
                abs(self._target_linear) <= 1.0e-6
                and abs(self._target_angular) <= 1.0e-6
            )
            if new_command_after_arm and neutral_command:
                self._post_arm_neutral_seen = True
            self._send_command(0, 0, True)
            return

        enable = self._enabled_requested and link_ok and remote_safe
        if not enable:
            self._send_command(0, 0, False)
            return

        left = (
            self._target_linear
            - 0.5 * self._wheel_separation * self._target_angular
        ) / self._wheel_radius
        right = (
            self._target_linear
            + 0.5 * self._wheel_separation * self._target_angular
        ) / self._wheel_radius
        left = max(-self._max_wheel_speed, min(self._max_wheel_speed, left))
        right = max(
            -self._max_wheel_speed, min(self._max_wheel_speed, right)
        )
        self._send_command(
            int(round(left * 1000.0)),
            int(round(right * 1000.0)),
            True,
        )

    def _send_command(
        self, left_mrad_s: int, right_mrad_s: int, enable: bool
    ) -> bool:
        payload = CommandPayload(
            left_mrad_s=left_mrad_s,
            right_mrad_s=right_mrad_s,
            ttl_ms=self._command_ttl_ms,
            enable=1 if enable else 0,
        ).pack()
        return self._write_frame(
            self._make_frame(PacketType.COMMAND, payload)
        )

    def _publish_telemetry(self, telemetry: TelemetryPayload) -> None:
        stamp = self._stamp()
        left_velocity = telemetry.velocity_left_mrad_s / 1000.0
        right_velocity = telemetry.velocity_right_mrad_s / 1000.0

        if self._last_encoder_left is not None:
            left_delta = _int32_delta(
                telemetry.encoder_left, self._last_encoder_left
            )
            right_delta = _int32_delta(
                telemetry.encoder_right, self._last_encoder_right
            )
            radians_per_tick = 2.0 * math.pi / self._ticks_per_revolution
            left_angle_delta = left_delta * radians_per_tick
            right_angle_delta = right_delta * radians_per_tick
            self._joint_left += left_angle_delta
            self._joint_right += right_angle_delta

            left_distance = left_angle_delta * self._wheel_radius
            right_distance = right_angle_delta * self._wheel_radius
            distance = 0.5 * (left_distance + right_distance)
            yaw_delta = (
                right_distance - left_distance
            ) / self._wheel_separation
            heading_midpoint = self._odom_yaw + 0.5 * yaw_delta
            self._odom_x += distance * math.cos(heading_midpoint)
            self._odom_y += distance * math.sin(heading_midpoint)
            self._odom_yaw = math.atan2(
                math.sin(self._odom_yaw + yaw_delta),
                math.cos(self._odom_yaw + yaw_delta),
            )

        self._last_encoder_left = telemetry.encoder_left
        self._last_encoder_right = telemetry.encoder_right

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [self._joint_name_left, self._joint_name_right]
        joints.position = [self._joint_left, self._joint_right]
        joints.velocity = [left_velocity, right_velocity]
        self._joint_pub.publish(joints)

        linear_velocity = (
            0.5 * (left_velocity + right_velocity) * self._wheel_radius
        )
        angular_velocity = (
            (right_velocity - left_velocity)
            * self._wheel_radius
            / self._wheel_separation
        )
        z, w = _quaternion_z(self._odom_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_odom
        odom.child_frame_id = self._frame_base
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = 1.0e6
        odom.pose.covariance[21] = 1.0e6
        odom.pose.covariance[28] = 1.0e6
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.03
        odom.twist.covariance[7] = 0.10
        odom.twist.covariance[14] = 1.0e6
        odom.twist.covariance[21] = 1.0e6
        odom.twist.covariance[28] = 1.0e6
        odom.twist.covariance[35] = 0.08
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._frame_odom
            transform.child_frame_id = self._frame_base
            transform.transform.translation.x = self._odom_x
            transform.transform.translation.y = self._odom_y
            transform.transform.rotation.z = z
            transform.transform.rotation.w = w
            self._tf_broadcaster.sendTransform(transform)

        self._publish_range(
            self._range_left_pub,
            self._frame_range_left,
            telemetry.range_left_mm,
            stamp,
        )
        self._publish_range(
            self._range_right_pub,
            self._frame_range_right,
            telemetry.range_right_mm,
            stamp,
        )
        self._publish_battery(telemetry, stamp)
        self._publish_status()

    def _publish_range(
        self,
        publisher,
        frame_id: str,
        millimetres: int,
        stamp,
    ) -> None:
        message = Range()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = self._range_fov
        message.min_range = self._range_min
        message.max_range = self._range_max
        if millimetres == 0xFFFF:
            message.range = float('nan')
        else:
            message.range = millimetres / 1000.0
        publisher.publish(message)

    def _publish_battery(self, telemetry: TelemetryPayload, stamp) -> None:
        message = BatteryState()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_base
        battery_valid = telemetry.battery_mv not in (0, 0xFFFF)
        message.voltage = (
            telemetry.battery_mv / 1000.0
            if battery_valid
            else float('nan')
        )
        current_valid = (
            telemetry.current_left_ma != -32768
            and telemetry.current_right_ma != -32768
        )
        message.current = (
            -(
                telemetry.current_left_ma + telemetry.current_right_ma
            ) / 1000.0
            if current_valid
            else float('nan')
        )
        message.present = battery_valid
        if battery_valid and self._battery_full > self._battery_empty:
            percentage = (
                (message.voltage - self._battery_empty)
                / (self._battery_full - self._battery_empty)
            )
            message.percentage = max(0.0, min(1.0, percentage))
        else:
            message.percentage = float('nan')
        self._battery_pub.publish(message)

    @staticmethod
    def _walker_constant(name: str, fallback: int) -> int:
        """Support the finalized interface and older development schemas."""

        return int(
            getattr(
                WalkerStatus,
                name,
                getattr(WalkerStatus, f'STATE_{name}', fallback),
            )
        )

    def _publish_status(self) -> None:
        now = self._now_monotonic()
        message = WalkerStatus()
        message.header.stamp = self._stamp()
        message.header.frame_id = self._frame_base
        message.link_ok = self._link_ok(now)
        message.session_id = self._session_id
        message.boot_id = self._boot_id
        message.crc_error_count = min(
            self._parser.crc_error_count, 0xFFFFFFFF
        )
        message.frame_error_count = min(
            self._parser.frame_error_count
            + self._payload_error_count
            + self._session_error_count
            + self._sequence_error_count,
            0xFFFFFFFF,
        )

        telemetry = self._last_telemetry
        if self._last_telemetry_time is None:
            message.telemetry_age = float('inf')
        else:
            message.telemetry_age = float(
                max(0.0, now - self._last_telemetry_time)
            )

        if telemetry is None:
            message.armed = False
            message.estop = False
            message.watchdog_timeout = False
            message.deadman = False
            message.fault_bits = 0
            message.last_applied_command_sequence = 0
            message.state = self._walker_constant(
                'DISCONNECTED',
                self._walker_constant('LINK_LOST', 0),
            )
            self._status_pub.publish(message)
            return

        firmware_state = self._firmware_state(telemetry)
        status_consistent = self._firmware_status_consistent(telemetry)
        message.armed = (
            bool(telemetry.status_bits & STATUS_MOTOR_ENABLED)
            and firmware_state == FW_ARMED
        )
        message.estop = bool(
            telemetry.status_bits & STATUS_ESTOP
        ) or firmware_state == FW_ESTOP
        message.watchdog_timeout = bool(
            telemetry.status_bits & STATUS_WATCHDOG_TIMEOUT
        )
        message.deadman = bool(telemetry.status_bits & STATUS_DEADMAN)
        message.fault_bits = telemetry.fault_bits
        if not status_consistent:
            message.fault_bits |= self._walker_constant(
                'FAULT_PROTOCOL',
                1 << 8,
            )
        message.last_applied_command_sequence = (
            telemetry.last_command_sequence
        )

        if not message.link_ok:
            message.state = self._walker_constant(
                'DISCONNECTED',
                self._walker_constant('LINK_LOST', 0),
            )
        elif message.estop:
            message.state = self._walker_constant('ESTOP', 4)
        elif (
            not status_consistent
            or telemetry.fault_bits
            or firmware_state == FW_FAULT
        ):
            message.state = self._walker_constant('FAULT', 5)
        elif (
            message.watchdog_timeout
            or firmware_state == FW_SAFE_STOP
        ):
            message.state = self._walker_constant(
                'SAFE_STOP',
                self._walker_constant('DISARMED', 1),
            )
        elif message.armed:
            message.state = self._walker_constant('ARMED', 2)
        else:
            message.state = self._walker_constant('DISARMED', 1)
        self._status_pub.publish(message)

    def _diagnostic_tick(self) -> None:
        self._publish_status()
        now = self._now_monotonic()
        array = DiagnosticArray()
        array.header.stamp = self._stamp()
        status = DiagnosticStatus()
        status.name = 'SafeStride serial bridge'
        status.hardware_id = (
            f'{self._port}/boot-{self._boot_id:08x}'
            if self._boot_id
            else self._port
        )

        if serial is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'pyserial is not installed'
        elif not self._serial_connected():
            status.level = DiagnosticStatus.ERROR
            status.message = 'serial port disconnected'
        elif not self._session_started:
            status.level = DiagnosticStatus.WARN
            status.message = 'waiting for controller HELLO'
        elif not self._link_ok(now):
            status.level = DiagnosticStatus.ERROR
            status.message = 'controller telemetry timed out'
        elif self._last_telemetry and self._last_telemetry.fault_bits:
            status.level = DiagnosticStatus.ERROR
            status.message = 'controller reports a fault'
        elif (
            self._last_telemetry
            and not self._firmware_status_consistent(
                self._last_telemetry
            )
        ):
            status.level = DiagnosticStatus.ERROR
            status.message = 'controller status fields are inconsistent'
        elif self._last_telemetry and (
            self._last_telemetry.status_bits & STATUS_ESTOP
        ):
            status.level = DiagnosticStatus.ERROR
            status.message = 'emergency stop is active'
        elif (
            self._parser.crc_error_count
            or self._parser.frame_error_count
            or self._payload_error_count
            or self._session_error_count
            or self._sequence_error_count
        ):
            status.level = DiagnosticStatus.WARN
            status.message = 'link active with malformed frames'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'link active'

        age = (
            float('inf')
            if self._last_telemetry_time is None
            else max(0.0, now - self._last_telemetry_time)
        )
        telemetry = self._last_telemetry
        status.values = [
            KeyValue(key='port', value=self._port),
            KeyValue(key='baudrate', value=str(self._baudrate)),
            KeyValue(key='session_id', value=f'0x{self._session_id:08x}'),
            KeyValue(key='boot_id', value=f'0x{self._boot_id:08x}'),
            KeyValue(key='telemetry_age_s', value=f'{age:.3f}'),
            KeyValue(
                key='crc_errors',
                value=str(self._parser.crc_error_count),
            ),
            KeyValue(
                key='frame_errors',
                value=str(
                    self._parser.frame_error_count
                    + self._payload_error_count
                    + self._session_error_count
                    + self._sequence_error_count
                ),
            ),
            KeyValue(
                key='fault_bits',
                value=(
                    f'0x{telemetry.fault_bits:04x}'
                    if telemetry is not None
                    else 'unknown'
                ),
            ),
            KeyValue(
                key='enabled_requested',
                value=str(self._enabled_requested).lower(),
            ),
            KeyValue(
                key='post_arm_neutral_seen',
                value=str(self._post_arm_neutral_seen).lower(),
            ),
        ]
        array.status = [status]
        self._diagnostics_pub.publish(array)

    def destroy_node(self) -> bool:
        """Best-effort disabled command before releasing the serial port."""

        if self._session_started and self._serial_connected():
            self._send_command(0, 0, False)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
