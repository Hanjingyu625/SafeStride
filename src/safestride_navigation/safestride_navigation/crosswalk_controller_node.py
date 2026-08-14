"""ROS 2 adapter for the automatic crosswalk v6 policy."""

import math
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

from safestride_interfaces.msg import CrosswalkStatus

from .crossing_policy import CrossingParameters, CrossingStateMachine
from .crosswalk_data import load_crosswalks, nearest_crosswalk
from .signal_logic import (
    DEFAULT_TIMING_URL,
    request_signal_data,
    signal_remaining_for_crosswalk,
)
from .speed_profile import UserSpeedProfile


STATE_VALUES = {
    'IDLE': CrosswalkStatus.STATE_IDLE,
    'APPROACHING': CrosswalkStatus.STATE_APPROACHING,
    'WAIT_AT_CURB': CrosswalkStatus.STATE_WAIT_AT_CURB,
    'ENTRY_ALLOWED': CrosswalkStatus.STATE_ENTRY_ALLOWED,
    'CROSSING': CrosswalkStatus.STATE_CROSSING,
    'CROSSING_URGENT': CrosswalkStatus.STATE_CROSSING_URGENT,
    'EXITING': CrosswalkStatus.STATE_EXITING,
}


def _finite_or_nan(value: Any) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


class CrosswalkController(Node):
    def __init__(self) -> None:
        super().__init__('crosswalk_controller')
        defaults = {
            'crosswalk_file': '',
            'api_key_file': '',
            'intersection_id': '',
            'signal_url': DEFAULT_TIMING_URL,
            'update_rate_hz': 5.0,
            'gps_timeout_s': 2.0,
            'speed_timeout_s': 1.0,
            'signal_refresh_interval_s': 3.0,
            'signal_cache_max_age_s': 12.0,
            'signal_request_timeout_s': 3.0,
            'maximum_crosswalk_distance_m': 80.0,
            'default_safe_speed_mps': 0.50,
            'profile_file': '',
            'motion_output_enabled': False,
            'fix_topic': '/gps/fix',
            'gps_speed_topic': '/gps/speed',
            'odom_topic': '/odom',
            'command_topic': '/cmd_vel',
            'status_topic': '/crosswalk/status',
            'diagnostics_topic': '/diagnostics',
            'output_frame_id': 'base_link',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        crosswalk_file = str(self.get_parameter('crosswalk_file').value)
        if not crosswalk_file or 'CHANGE_ME' in crosswalk_file:
            raise ValueError(
                'crosswalk_file must point to generated standard_crosswalks.json'
            )
        self._crosswalks = load_crosswalks(crosswalk_file)
        self._intersection_id = str(
            self.get_parameter('intersection_id').value
        ).strip()
        self._signal_url = str(self.get_parameter('signal_url').value)
        self._api_key = self._load_api_key(
            str(self.get_parameter('api_key_file').value)
        )
        self._gps_timeout = self._positive('gps_timeout_s')
        self._speed_timeout = self._positive('speed_timeout_s')
        self._signal_refresh = self._positive(
            'signal_refresh_interval_s'
        )
        self._signal_cache_max_age = self._positive(
            'signal_cache_max_age_s'
        )
        self._signal_request_timeout = self._positive(
            'signal_request_timeout_s'
        )
        self._maximum_crosswalk_distance = self._positive(
            'maximum_crosswalk_distance_m'
        )
        update_rate = self._positive('update_rate_hz')
        default_safe_speed = self._positive('default_safe_speed_mps')
        self._motion_output_enabled = bool(
            self.get_parameter('motion_output_enabled').value
        )
        self._output_frame_id = str(
            self.get_parameter('output_frame_id').value
        )

        self._controller = CrossingStateMachine(CrossingParameters())
        self._profile = UserSpeedProfile(
            str(self.get_parameter('profile_file').value),
            default_speed_mps=default_safe_speed,
        )
        self._fix: Optional[Tuple[float, float]] = None
        self._fix_time: Optional[float] = None
        self._gps_speed: Optional[float] = None
        self._gps_speed_time: Optional[float] = None
        self._odom_speed: Optional[float] = None
        self._odom_time: Optional[float] = None
        self._odom_position: Optional[Tuple[float, float]] = None
        self._wheel_distance_m = 0.0

        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='crosswalk-signal',
        )
        self._signal_future: Optional[Future] = None
        self._signal_future_id = ''
        self._signal_cache: Optional[Mapping[str, Any]] = None
        self._signal_cache_id = ''
        self._signal_cache_time: Optional[float] = None
        self._last_signal_request = -math.inf
        self._signal_error = ''
        self._last_diagnostic = -math.inf
        self._last_summary = ''

        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            self._fix_callback,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('gps_speed_topic').value),
            self._gps_speed_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            10,
        )
        self._command_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('command_topic').value),
            10,
        )
        self._status_publisher = self.create_publisher(
            CrosswalkStatus,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self._timer = self.create_timer(1.0 / update_rate, self._tick)
        self.get_logger().info(
            'Loaded %d crosswalks; motion output is %s'
            % (
                len(self._crosswalks),
                'ENABLED' if self._motion_output_enabled else 'monitor-only',
            )
        )
        if not self._api_key:
            self.get_logger().warning(
                'No signal API key: curb entry remains fail-safe stopped'
            )

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(name + ' must be finite and positive')
        return value

    @staticmethod
    def _load_api_key(path: str) -> str:
        if not path or 'CHANGE_ME' in path:
            return ''
        source = Path(path).expanduser()
        try:
            return source.read_text(encoding='utf-8-sig').strip()
        except OSError:
            return ''

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _fresh(now: float, received_at: Optional[float], timeout: float) -> bool:
        return (
            received_at is not None
            and now >= received_at
            and now - received_at <= timeout
        )

    def _fix_callback(self, message: NavSatFix) -> None:
        latitude = float(message.latitude)
        longitude = float(message.longitude)
        valid = (
            message.status.status != NavSatStatus.STATUS_NO_FIX
            and math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )
        if valid:
            self._fix = (latitude, longitude)
            self._fix_time = self._now()

    def _gps_speed_callback(self, message: Float32) -> None:
        speed = float(message.data)
        if math.isfinite(speed) and 0.0 <= speed <= 3.0:
            self._gps_speed = speed
            self._gps_speed_time = self._now()

    def _odom_callback(self, message: Odometry) -> None:
        now = self._now()
        linear = message.twist.twist.linear
        speed = math.hypot(float(linear.x), float(linear.y))
        if math.isfinite(speed) and 0.0 <= speed <= 3.0:
            self._odom_speed = speed
            self._odom_time = now

        position = message.pose.pose.position
        current = (float(position.x), float(position.y))
        if not all(math.isfinite(value) for value in current):
            return
        if self._odom_position is not None:
            step = math.hypot(
                current[0] - self._odom_position[0],
                current[1] - self._odom_position[1],
            )
            if 0.0 <= step <= 2.0:
                self._wheel_distance_m += step
        self._odom_position = current

    def _measured_speed(self, now: float) -> Optional[float]:
        if self._fresh(now, self._odom_time, self._speed_timeout):
            return self._odom_speed
        if self._fresh(now, self._gps_speed_time, self._speed_timeout):
            return self._gps_speed
        return None

    def _consume_signal_future(self, now: float) -> None:
        future = self._signal_future
        if future is None or not future.done():
            return
        requested_id = self._signal_future_id
        self._signal_future = None
        self._signal_future_id = ''
        try:
            self._signal_cache = future.result()
            self._signal_cache_id = requested_id
            self._signal_cache_time = now
            self._signal_error = ''
        except Exception as error:
            self._signal_error = str(error)

    def _request_signal_if_due(self, intersection_id: str, now: float) -> None:
        if (
            not self._api_key
            or not intersection_id
            or self._signal_future is not None
            or now - self._last_signal_request < self._signal_refresh
        ):
            return
        self._last_signal_request = now
        self._signal_future_id = intersection_id
        self._signal_future = self._executor.submit(
            request_signal_data,
            self._api_key,
            intersection_id,
            url=self._signal_url,
            timeout_s=self._signal_request_timeout,
        )

    def _signal_state(
        self,
        intersection_id: str,
        direction: str,
        now: float,
    ) -> Tuple[Optional[float], bool, str]:
        self._request_signal_if_due(intersection_id, now)
        if (
            self._signal_cache is None
            or self._signal_cache_id != intersection_id
            or not self._fresh(
                now,
                self._signal_cache_time,
                self._signal_cache_max_age,
            )
        ):
            reason = self._signal_error or 'signal data unavailable or stale'
            return None, False, reason
        try:
            (remaining, _field), _raw = signal_remaining_for_crosswalk(
                self._signal_cache,
                direction,
            )
            return remaining, True, ''
        except ValueError as error:
            return None, False, str(error)

    def _publish_command(self, speed_mps: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._output_frame_id
        message.twist.linear.x = speed_mps
        self._command_publisher.publish(message)

    def _publish_status(
        self,
        *,
        active: Optional[Mapping[str, Any]],
        gps_valid: bool,
        signal_valid: bool,
        signal_remaining_s: Optional[float],
        required_entry_s: Optional[float],
        crossing_eta_s: Optional[float],
        command: Mapping[str, Any],
        effective_speed_mps: float,
        intersection_id: str,
    ) -> None:
        status = CrosswalkStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'map'
        status.state = STATE_VALUES[self._controller.state]
        status.gps_valid = gps_valid
        status.signal_valid = signal_valid
        status.entry_allowed = bool(command['entry_allowed'])
        status.urgent = self._controller.state == 'CROSSING_URGENT'
        status.edge_distance_m = _finite_or_nan(
            active.get('edge_distance_m') if active else None
        )
        status.progress_m = _finite_or_nan(
            active.get('progress_m') if active else None
        )
        status.remaining_m = _finite_or_nan(
            active.get('remaining_m') if active else None
        )
        status.signal_remaining_s = _finite_or_nan(signal_remaining_s)
        status.required_entry_s = _finite_or_nan(required_entry_s)
        status.crossing_eta_s = _finite_or_nan(crossing_eta_s)
        status.target_speed_mps = effective_speed_mps
        status.command_mode = str(command['mode'])
        status.reason = self._controller.reason
        status.intersection_id = intersection_id
        status.crosswalk_index = int(active.get('index', 0) if active else 0)
        self._status_publisher.publish(status)

    def _publish_diagnostic(
        self,
        now: float,
        *,
        gps_valid: bool,
        signal_valid: bool,
        signal_reason: str,
        intersection_id: str,
        effective_speed_mps: float,
    ) -> None:
        if now - self._last_diagnostic < 1.0:
            return
        self._last_diagnostic = now
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'SafeStride/Crosswalk Controller'
        diagnostic.hardware_id = 'BE-220/V2X'
        if not gps_valid:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'GPS unavailable or stale'
        elif self._controller.state in ('WAIT_AT_CURB', 'ENTRY_ALLOWED') and not signal_valid:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = signal_reason
        elif not self._motion_output_enabled:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = 'monitor-only; motion output disabled'
        else:
            diagnostic.level = DiagnosticStatus.OK
            diagnostic.message = self._controller.reason
        diagnostic.values = [
            KeyValue(key='state', value=self._controller.state),
            KeyValue(key='intersection_id', value=intersection_id or 'unset'),
            KeyValue(key='gps_valid', value=str(gps_valid).lower()),
            KeyValue(key='signal_valid', value=str(signal_valid).lower()),
            KeyValue(
                key='motion_output_enabled',
                value=str(self._motion_output_enabled).lower(),
            ),
            KeyValue(
                key='effective_speed_mps',
                value='%.3f' % effective_speed_mps,
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self._diagnostic_publisher.publish(array)

    def _tick(self) -> None:
        now = self._now()
        self._consume_signal_future(now)
        gps_valid = self._fix is not None and self._fresh(
            now,
            self._fix_time,
            self._gps_timeout,
        )
        active = None
        required_entry_s = None
        crossing_eta_s = None
        signal_remaining_s = None
        signal_valid = False
        signal_reason = ''
        intersection_id = self._controller.locked_intersection_id
        measured_speed = self._measured_speed(now)

        if gps_valid:
            assert self._fix is not None
            latitude, longitude = self._fix
            candidate = nearest_crosswalk(
                self._crosswalks,
                latitude,
                longitude,
            )
            if (
                candidate is not None
                and candidate['edge_distance_m']
                > self._maximum_crosswalk_distance
                and self._controller.locked_crosswalk is None
            ):
                candidate = None
            preview = self._controller.current_crosswalk(
                candidate,
                latitude,
                longitude,
            )
            intersection_id = (
                self._controller.locked_intersection_id
                or str((preview or {}).get('intersection_id') or '')
                or self._intersection_id
            )
            if preview is not None:
                signal_remaining_s, signal_valid, signal_reason = (
                    self._signal_state(
                        intersection_id,
                        str(preview['signal_direction']),
                        now,
                    )
                )
            safe_speed = self._profile.safe_speed()
            active, required_entry_s, crossing_eta_s = self._controller.update(
                candidate=candidate,
                intersection_id=intersection_id,
                latitude=latitude,
                longitude=longitude,
                signal_remaining_s=signal_remaining_s,
                signal_valid=signal_valid,
                safe_speed_mps=safe_speed,
                measured_speed_mps=measured_speed,
                wheel_distance_m=self._wheel_distance_m,
            )
            self._profile.add(
                measured_speed,
                allow_update=self._controller.state
                not in ('WAIT_AT_CURB', 'CROSSING_URGENT'),
            )
        else:
            self._controller.reset('GPS unavailable or stale')
            safe_speed = self._profile.safe_speed()

        command = self._controller.command(safe_speed, measured_speed)
        desired_speed = float(command['target_speed_mps']) if gps_valid else 0.0
        effective_speed = (
            desired_speed if self._motion_output_enabled else 0.0
        )
        self._publish_command(effective_speed)
        self._publish_status(
            active=active,
            gps_valid=gps_valid,
            signal_valid=signal_valid,
            signal_remaining_s=signal_remaining_s,
            required_entry_s=required_entry_s,
            crossing_eta_s=crossing_eta_s,
            command=command,
            effective_speed_mps=effective_speed,
            intersection_id=intersection_id,
        )
        self._publish_diagnostic(
            now,
            gps_valid=gps_valid,
            signal_valid=signal_valid,
            signal_reason=signal_reason,
            intersection_id=intersection_id,
            effective_speed_mps=effective_speed,
        )

    def destroy_node(self) -> bool:
        self._publish_command(0.0)
        try:
            self._profile.save()
        except OSError as error:
            self.get_logger().warning('Could not save speed profile: %s' % error)
        if self._signal_future is not None:
            self._signal_future.cancel()
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrosswalkController()
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
