"""ROS 2 adapter for the automatic crosswalk v6 policy."""

import math
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Any, Mapping, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

from safestride_interfaces.msg import CrosswalkStatus

from .crossing_policy import CrossingParameters, CrossingStateMachine
from .crosswalk_data import (
    CrosswalkSpatialIndex,
    load_crosswalks,
)
from .gps_motion import GpsMotionTracker
from .intersection_map import (
    DEFAULT_INTERSECTION_MAP_URL,
    load_intersection_map,
    nearest_intersection,
    request_intersection_map,
    save_intersection_map,
    select_intersection_id,
)
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
            'intersection_map_url': DEFAULT_INTERSECTION_MAP_URL,
            'intersection_map_page_size': 100,
            'intersection_map_max_pages': 30,
            'intersection_map_retry_s': 60.0,
            'intersection_map_file': '',
            'intersection_map_cache_file': (
                '~/.cache/safestride/v2x_intersections.json'
            ),
            'maximum_intersection_distance_m': 120.0,
            'update_rate_hz': 5.0,
            'gps_timeout_s': 2.0,
            'speed_timeout_s': 1.0,
            'heading_timeout_s': 5.0,
            'heading_min_move_m': 2.0,
            'heading_max_step_m': 30.0,
            'candidate_heading_tolerance_deg': 60.0,
            'gps_change_threshold_m': 0.5,
            'gps_stuck_timeout_s': 5.0,
            'gps_stuck_speed_mps': 0.15,
            'signal_refresh_interval_s': 3.0,
            'signal_cache_max_age_s': 12.0,
            'signal_request_timeout_s': 3.0,
            'maximum_crosswalk_distance_m': 80.0,
            'default_safe_speed_mps': 0.50,
            'profile_file': '',
            'motion_output_enabled': False,
            'fix_topic': '/gps/fix',
            'gps_speed_topic': '/gps/speed',
            'gps_course_topic': '/gps/course',
            'odom_topic': '/odom',
            'command_topic': '/cmd_vel',
            'status_topic': '/crosswalk/status',
            'diagnostics_topic': '/diagnostics',
            'output_frame_id': 'base_link',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        crosswalk_file = str(self.get_parameter('crosswalk_file').value)
        self._crosswalks = []
        self._crosswalk_index: Optional[CrosswalkSpatialIndex] = None
        self._map_ready = False
        self._map_error = 'crosswalk map file is not configured'
        if crosswalk_file and 'CHANGE_ME' not in crosswalk_file:
            try:
                self._crosswalks = load_crosswalks(crosswalk_file)
                self._map_ready = bool(self._crosswalks)
                if self._map_ready:
                    self._crosswalk_index = CrosswalkSpatialIndex(
                        self._crosswalks
                    )
                self._map_error = (
                    '' if self._map_ready else 'crosswalk map is empty'
                )
            except (OSError, ValueError, TypeError) as error:
                self._map_error = 'crosswalk map unavailable: %s' % error
        self._intersection_id = str(
            self.get_parameter('intersection_id').value
        ).strip()
        self._signal_url = str(self.get_parameter('signal_url').value)
        self._intersection_map_url = str(
            self.get_parameter('intersection_map_url').value
        )
        self._intersection_map_page_size = int(
            self.get_parameter('intersection_map_page_size').value
        )
        self._intersection_map_max_pages = int(
            self.get_parameter('intersection_map_max_pages').value
        )
        if (
            self._intersection_map_page_size <= 0
            or self._intersection_map_max_pages <= 0
        ):
            raise ValueError('intersection map pagination must be positive')
        self._api_key = self._load_api_key(
            str(self.get_parameter('api_key_file').value)
        )
        self._api_ready = bool(self._api_key)
        cache_file = str(
            self.get_parameter('intersection_map_cache_file').value
        ).strip()
        intersection_map_file = str(
            self.get_parameter('intersection_map_file').value
        ).strip()
        self._intersection_map_file = (
            Path(intersection_map_file).expanduser()
            if intersection_map_file
            else None
        )
        self._intersection_map_cache_file = (
            Path(cache_file).expanduser() if cache_file else None
        )
        self._gps_timeout = self._positive('gps_timeout_s')
        self._speed_timeout = self._positive('speed_timeout_s')
        self._heading_timeout = self._positive('heading_timeout_s')
        self._heading_min_move = self._positive('heading_min_move_m')
        self._heading_max_step = self._positive('heading_max_step_m')
        self._heading_tolerance = self._positive(
            'candidate_heading_tolerance_deg'
        )
        if self._heading_tolerance > 180.0:
            raise ValueError(
                'candidate_heading_tolerance_deg must not exceed 180'
            )
        self._gps_change_threshold = self._positive(
            'gps_change_threshold_m'
        )
        self._gps_stuck_timeout = self._positive('gps_stuck_timeout_s')
        self._gps_stuck_speed = self._nonnegative('gps_stuck_speed_mps')
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
        self._maximum_intersection_distance = self._positive(
            'maximum_intersection_distance_m'
        )
        self._intersection_map_retry = self._positive(
            'intersection_map_retry_s'
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
        self._gps_motion = GpsMotionTracker(
            change_threshold_m=self._gps_change_threshold,
            heading_min_move_m=self._heading_min_move,
            heading_max_step_m=self._heading_max_step,
        )
        self._gps_speed: Optional[float] = None
        self._gps_speed_time: Optional[float] = None
        self._odom_speed: Optional[float] = None
        self._odom_time: Optional[float] = None
        self._odom_position: Optional[Tuple[float, float]] = None
        self._wheel_distance_m = 0.0

        self._executor = ThreadPoolExecutor(
            max_workers=2,
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
        self._last_selection_key = None

        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('gps_speed_topic').value),
            self._gps_speed_callback,
            qos_profile_sensor_data,
        )
        self._intersections = []
        self._intersection_map_future: Optional[Future] = None
        self._intersection_map_error = ''
        self._intersection_map_origin = 'none'
        self._nearest_intersection: Optional[Mapping[str, Any]] = None
        self._intersection_match_cache = {}
        self._last_intersection_map_request = -math.inf
        if self._intersection_map_cache_file is not None:
            try:
                self._intersections = load_intersection_map(
                    self._intersection_map_cache_file
                )
                self._intersection_map_origin = 'cache'
                self.get_logger().info(
                    'Loaded %d cached V2X intersections from %s'
                    % (
                        len(self._intersections),
                        self._intersection_map_cache_file,
                    )
                )
            except FileNotFoundError:
                pass
            except (OSError, ValueError, TypeError) as error:
                self._intersection_map_error = (
                    'cached V2X map unavailable: %s' % error
                )
        if (
            not self._intersections
            and self._intersection_map_file is not None
        ):
            try:
                self._intersections = load_intersection_map(
                    self._intersection_map_file
                )
                self._intersection_map_origin = 'offline_file'
                self.get_logger().info(
                    'Loaded %d offline V2X intersections from %s'
                    % (
                        len(self._intersections),
                        self._intersection_map_file,
                    )
                )
            except (OSError, ValueError, TypeError) as error:
                self._intersection_map_error = (
                    'offline V2X map unavailable: %s' % error
                )
        self.create_subscription(
            Float32,
            str(self.get_parameter('gps_course_topic').value),
            self._gps_course_callback,
            qos_profile_sensor_data,
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
                'No signal API key: readiness monitor remains active'
            )
        if not self._map_ready:
            self.get_logger().warning(self._map_error)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(name + ' must be finite and positive')
        return value

    def _nonnegative(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(name + ' must be finite and nonnegative')
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
    def _fresh(
        now: float,
        received_at: Optional[float],
        timeout: float,
    ) -> bool:
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
            now = self._now()
            current = (latitude, longitude)
            self._gps_motion.update(latitude, longitude, now)
            self._fix = current
            self._fix_time = now

    def _gps_speed_callback(self, message: Float32) -> None:
        speed = float(message.data)
        if math.isfinite(speed) and 0.0 <= speed <= 3.0:
            self._gps_speed = speed
            self._gps_speed_time = self._now()

    def _gps_course_callback(self, message: Float32) -> None:
        course = float(message.data)
        if math.isfinite(course) and 0.0 <= course <= 360.0:
            self._gps_motion.set_course(course, self._now())

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

    def _heading(self, now: float) -> Optional[float]:
        return self._gps_motion.heading(now, self._heading_timeout)

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

    def _consume_intersection_map_future(self) -> None:
        future = self._intersection_map_future
        if future is None or not future.done():
            return
        self._intersection_map_future = None
        try:
            self._intersections = future.result()
            self._intersection_match_cache.clear()
            self._intersection_map_error = ''
            self._intersection_map_origin = 'api'
            self.get_logger().info(
                'Loaded %d V2X intersections' % len(self._intersections)
            )
            if self._intersection_map_cache_file is not None:
                try:
                    save_intersection_map(
                        self._intersection_map_cache_file,
                        self._intersections,
                    )
                except (OSError, ValueError, TypeError) as error:
                    self.get_logger().warning(
                        'Could not cache V2X intersection map: %s' % error
                    )
        except Exception as error:
            self._intersection_map_error = str(error)
            self.get_logger().warning(
                'V2X intersection map unavailable: %s' % error
            )

    def _request_intersection_map_if_due(self, now: float) -> None:
        if (
            not self._api_key
            or self._intersection_map_origin in ('api', 'cache')
            or self._intersection_map_future is not None
            or now - self._last_intersection_map_request
            < self._intersection_map_retry
        ):
            return
        self._last_intersection_map_request = now
        self._intersection_map_future = self._executor.submit(
            request_intersection_map,
            self._api_key,
            url=self._intersection_map_url,
            page_size=self._intersection_map_page_size,
            max_pages=self._intersection_map_max_pages,
            timeout_s=self._signal_request_timeout,
        )

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
        target_speed_mps: float,
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
        status.target_speed_mps = target_speed_mps
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
        intersection_source: str,
        intersection_name: str,
        effective_speed_mps: float,
        active: Optional[Mapping[str, Any]],
        heading: Optional[float],
        gps_stuck: bool,
    ) -> None:
        if now - self._last_diagnostic < 1.0:
            return
        self._last_diagnostic = now
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'SafeStride/Crosswalk Controller'
        diagnostic.hardware_id = 'BE-220/V2X'
        if not self._map_ready:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = self._map_error
        elif not gps_valid:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = (
                'GPS coordinates are not changing while motion is reported'
                if gps_stuck
                else 'GPS unavailable or stale'
            )
        elif not self._api_ready:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = 'signal API key is not configured'
        elif intersection_source == 'configured_fallback':
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = (
                'using configured intersection fallback; dynamic match '
                'is unavailable'
            )
        elif active is not None and not intersection_id:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = (
                self._intersection_map_error
                or (
                    'V2X intersection map is loading'
                    if self._intersection_map_future is not None
                    else ''
                )
                or 'no V2X intersection matched the selected crosswalk'
            )
        elif (
            self._controller.state in ('WAIT_AT_CURB', 'ENTRY_ALLOWED')
            and not signal_valid
        ):
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
            KeyValue(
                key='intersection_name', value=intersection_name or 'unknown'
            ),
            KeyValue(key='intersection_id_source', value=intersection_source),
            KeyValue(
                key='configured_intersection_id',
                value=self._intersection_id or 'unset',
            ),
            KeyValue(key='gps_valid', value=str(gps_valid).lower()),
            KeyValue(key='gps_stuck', value=str(gps_stuck).lower()),
            KeyValue(
                key='latitude',
                value=(
                    '%.8f' % self._fix[0]
                    if self._fix is not None
                    else 'nan'
                ),
            ),
            KeyValue(
                key='longitude',
                value=(
                    '%.8f' % self._fix[1]
                    if self._fix is not None
                    else 'nan'
                ),
            ),
            KeyValue(
                key='heading_deg',
                value='%.1f' % heading if heading is not None else 'nan',
            ),
            KeyValue(
                key='heading_source', value=self._gps_motion.heading_source
            ),
            KeyValue(
                key='crosswalk_index',
                value=str(active.get('index')) if active else 'none',
            ),
            KeyValue(
                key='crosswalk_edge_distance_m',
                value=(
                    '%.2f' % float(active['edge_distance_m'])
                    if active is not None
                    else 'nan'
                ),
            ),
            KeyValue(
                key='candidate_heading_error_deg',
                value=(
                    '%.1f' % float(active['heading_error_deg'])
                    if active is not None
                    and math.isfinite(float(active['heading_error_deg']))
                    else 'nan'
                ),
            ),
            KeyValue(
                key='target_bearing_deg',
                value=(
                    '%.1f' % float(active['target_bearing_deg'])
                    if active is not None
                    else 'nan'
                ),
            ),
            KeyValue(
                key='crossing_direction',
                value=str(active.get('crossing_direction', 'none'))
                if active
                else 'none',
            ),
            KeyValue(
                key='signal_direction',
                value=str(active.get('signal_direction', 'none'))
                if active
                else 'none',
            ),
            KeyValue(
                key='search_candidate_count',
                value=str(active.get('search_candidate_count', 0))
                if active
                else '0',
            ),
            KeyValue(key='map_ready', value=str(self._map_ready).lower()),
            KeyValue(key='api_ready', value=str(self._api_ready).lower()),
            KeyValue(
                key='intersection_map_count',
                value=str(len(self._intersections)),
            ),
            KeyValue(
                key='intersection_map_origin',
                value=self._intersection_map_origin,
            ),
            KeyValue(
                key='intersection_distance_m',
                value=(
                    '%.1f' % float(self._nearest_intersection['distance_m'])
                    if self._nearest_intersection is not None
                    else 'nan'
                ),
            ),
            KeyValue(
                key='crosswalk_count', value=str(len(self._crosswalks))
            ),
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
        self._consume_intersection_map_future()
        self._request_intersection_map_if_due(now)
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
        intersection_id, intersection_source, intersection_name = (
            select_intersection_id(
                locked_id=self._controller.locked_intersection_id,
                crosswalk=None,
                nearest=None,
                configured_id='',
            )
        )
        measured_speed = self._measured_speed(now)
        heading = self._heading(now)
        self._nearest_intersection = None
        gps_stuck = gps_valid and self._gps_motion.coordinates_stuck(
            now,
            measured_speed,
            timeout_s=self._gps_stuck_timeout,
            minimum_speed_mps=self._gps_stuck_speed,
        )
        if gps_stuck:
            gps_valid = False

        if gps_valid:
            assert self._fix is not None
            latitude, longitude = self._fix
            candidate = (
                self._crosswalk_index.nearest(
                    latitude,
                    longitude,
                    maximum_distance_m=self._maximum_crosswalk_distance,
                    heading_deg=heading,
                    maximum_heading_error_deg=self._heading_tolerance,
                )
                if self._crosswalk_index is not None
                else None
            )
            preview = self._controller.current_crosswalk(
                candidate,
                latitude,
                longitude,
            )
            if preview is not None and self._intersections:
                crosswalk_index = int(preview['index'])
                if crosswalk_index not in self._intersection_match_cache:
                    self._intersection_match_cache[crosswalk_index] = (
                        nearest_intersection(
                            self._intersections,
                            float(preview['latitude']),
                            float(preview['longitude']),
                            maximum_distance_m=(
                                self._maximum_intersection_distance
                            ),
                        )
                    )
                self._nearest_intersection = (
                    self._intersection_match_cache[crosswalk_index]
                )
            intersection_id, intersection_source, intersection_name = (
                select_intersection_id(
                    locked_id=self._controller.locked_intersection_id,
                    crosswalk=preview,
                    nearest=self._nearest_intersection,
                    configured_id=(
                        self._intersection_id if not self._api_ready else ''
                    ),
                )
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

        selection_key = (
            int(active['index']) if active is not None else None,
            intersection_id,
            intersection_source,
        )
        if gps_valid and selection_key != self._last_selection_key:
            assert self._fix is not None
            self.get_logger().info(
                'GPS %.8f,%.8f -> crosswalk %s (%.1f m) -> '
                'intersection %s %s [%s]'
                % (
                    self._fix[0],
                    self._fix[1],
                    active.get('index') if active is not None else 'none',
                    float(active['edge_distance_m'])
                    if active is not None
                    else math.nan,
                    intersection_id or 'none',
                    intersection_name or 'unknown',
                    intersection_source,
                )
            )
            self._last_selection_key = selection_key

        command = self._controller.command(safe_speed, measured_speed)
        desired_speed = (
            float(command['target_speed_mps']) if gps_valid else 0.0
        )
        effective_speed = (
            desired_speed if self._motion_output_enabled else 0.0
        )
        if self._motion_output_enabled:
            self._publish_command(effective_speed)
        self._publish_status(
            active=active,
            gps_valid=gps_valid,
            signal_valid=signal_valid,
            signal_remaining_s=signal_remaining_s,
            required_entry_s=required_entry_s,
            crossing_eta_s=crossing_eta_s,
            command=command,
            target_speed_mps=desired_speed,
            intersection_id=intersection_id,
        )
        self._publish_diagnostic(
            now,
            gps_valid=gps_valid,
            signal_valid=signal_valid,
            signal_reason=signal_reason,
            intersection_id=intersection_id,
            intersection_source=intersection_source,
            intersection_name=intersection_name,
            effective_speed_mps=effective_speed,
            active=active,
            heading=heading,
            gps_stuck=gps_stuck,
        )

    def destroy_node(self) -> bool:
        if self._motion_output_enabled:
            self._publish_command(0.0)
        try:
            self._profile.save()
        except OSError as error:
            self.get_logger().warning(
                'Could not save speed profile: %s' % error
            )
        if self._signal_future is not None:
            self._signal_future.cancel()
        if self._intersection_map_future is not None:
            self._intersection_map_future.cancel()
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
