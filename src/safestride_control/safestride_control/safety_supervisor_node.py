"""Fail-safe velocity-command supervisor for the SafeStride walker."""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Range

from safestride_interfaces.msg import (
    SurfaceCondition,
    TerrainStatus,
    WalkerStatus,
)
from .safety_logic import finite_parameter


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _bool_text(value: bool) -> str:
    return 'true' if value else 'false'


class SafetySupervisor(Node):
    """Validate and limit motion commands before they reach the MCU bridge."""

    def __init__(self) -> None:
        super().__init__('safety_supervisor')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('diagnostic_rate', 2.0)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('status_timeout', 0.50)
        self.declare_parameter('max_telemetry_age', 0.50)
        self.declare_parameter('range_timeout', 0.35)
        self.declare_parameter('surface_timeout', 2.5)
        self.declare_parameter('require_range_sensors', True)
        self.declare_parameter('require_surface_condition', False)
        self.declare_parameter('require_deadman', True)

        self.declare_parameter('max_forward_velocity', 0.15)
        self.declare_parameter('max_reverse_velocity', 0.08)
        self.declare_parameter('max_angular_velocity', 0.35)
        self.declare_parameter('max_surface_speed_scale', 1.25)
        self.declare_parameter('max_linear_acceleration', 0.20)
        self.declare_parameter('max_linear_deceleration', 0.50)
        self.declare_parameter('max_angular_acceleration', 0.50)
        self.declare_parameter('max_angular_deceleration', 1.00)

        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.80)

        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('safe_command_topic', '/cmd_vel_safe')
        self.declare_parameter('left_range_topic', '/range/front_left')
        self.declare_parameter('right_range_topic', '/range/front_right')
        self.declare_parameter(
            'surface_topic', '/perception/surface_condition'
        )
        self.declare_parameter('status_topic', '/walker/status')
        self.declare_parameter('terrain_status_topic', '/terrain/status')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('output_frame_id', 'base_link')

        self._publish_rate = finite_parameter(
            'publish_rate',
            self.get_parameter('publish_rate').value,
            minimum=0.0,
            maximum=200.0,
            minimum_inclusive=False,
        )
        self._diagnostic_rate = finite_parameter(
            'diagnostic_rate',
            self.get_parameter('diagnostic_rate').value,
            minimum=0.0,
            maximum=100.0,
            minimum_inclusive=False,
        )
        self._command_timeout = finite_parameter(
            'command_timeout',
            self.get_parameter('command_timeout').value,
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._status_timeout = finite_parameter(
            'status_timeout',
            self.get_parameter('status_timeout').value,
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._max_telemetry_age = finite_parameter(
            'max_telemetry_age',
            self.get_parameter('max_telemetry_age').value,
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._range_timeout = finite_parameter(
            'range_timeout',
            self.get_parameter('range_timeout').value,
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._surface_timeout = finite_parameter(
            'surface_timeout',
            self.get_parameter('surface_timeout').value,
            minimum=0.0,
            maximum=10.0,
            minimum_inclusive=False,
        )
        self._require_ranges = bool(
            self.get_parameter('require_range_sensors').value
        )
        self._require_surface = bool(
            self.get_parameter('require_surface_condition').value
        )
        self._require_deadman = bool(
            self.get_parameter('require_deadman').value
        )
        self._max_forward = finite_parameter(
            'max_forward_velocity',
            self.get_parameter('max_forward_velocity').value,
            minimum=0.0,
            maximum=10.0,
        )
        self._max_reverse = finite_parameter(
            'max_reverse_velocity',
            self.get_parameter('max_reverse_velocity').value,
            minimum=0.0,
            maximum=10.0,
        )
        self._max_angular = finite_parameter(
            'max_angular_velocity',
            self.get_parameter('max_angular_velocity').value,
            minimum=0.0,
            maximum=20.0,
        )
        self._max_surface_scale = finite_parameter(
            'max_surface_speed_scale',
            self.get_parameter('max_surface_speed_scale').value,
            minimum=1.0,
            maximum=2.0,
        )
        self._linear_accel = finite_parameter(
            'max_linear_acceleration',
            self.get_parameter('max_linear_acceleration').value,
            minimum=0.0,
            maximum=20.0,
        )
        self._linear_decel = finite_parameter(
            'max_linear_deceleration',
            self.get_parameter('max_linear_deceleration').value,
            minimum=0.0,
            maximum=20.0,
            minimum_inclusive=False,
        )
        self._angular_accel = finite_parameter(
            'max_angular_acceleration',
            self.get_parameter('max_angular_acceleration').value,
            minimum=0.0,
            maximum=50.0,
        )
        self._angular_decel = finite_parameter(
            'max_angular_deceleration',
            self.get_parameter('max_angular_deceleration').value,
            minimum=0.0,
            maximum=50.0,
            minimum_inclusive=False,
        )

        self._stop_distance = finite_parameter(
            'stop_distance',
            self.get_parameter('stop_distance').value,
            minimum=0.0,
            maximum=100.0,
            minimum_inclusive=False,
        )
        self._slow_distance = finite_parameter(
            'slow_distance',
            self.get_parameter('slow_distance').value,
            minimum=0.0,
            maximum=100.0,
            minimum_inclusive=False,
        )
        if self._slow_distance <= self._stop_distance:
            raise ValueError(
                'slow_distance must exceed stop_distance'
            )

        self._output_frame_id = str(
            self.get_parameter('output_frame_id').value
        )

        self._last_command: Optional[TwistStamped] = None
        self._last_command_time: Optional[float] = None
        self._last_status: Optional[WalkerStatus] = None
        self._last_status_time: Optional[float] = None
        self._last_terrain: Optional[TerrainStatus] = None
        self._last_terrain_time: Optional[float] = None
        self._last_surface: Optional[SurfaceCondition] = None
        self._last_surface_time: Optional[float] = None
        self._ranges: Dict[str, Dict[str, object]] = {
            'left': {
                'distance': math.nan,
                'valid': False,
                'time': None,
            },
            'right': {
                'distance': math.nan,
                'valid': False,
                'time': None,
            },
        }

        self._output_linear = 0.0
        self._output_angular = 0.0
        self._last_tick_time = self._now_seconds()
        self._last_diagnostic_time = -math.inf
        self._last_log_summary = ''
        self._command_output_suppressed = False

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            TwistStamped,
            str(self.get_parameter('command_topic').value),
            self._command_callback,
            10,
        )
        self.create_subscription(
            Range,
            str(self.get_parameter('left_range_topic').value),
            lambda msg: self._range_callback('left', msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            str(self.get_parameter('right_range_topic').value),
            lambda msg: self._range_callback('right', msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WalkerStatus,
            str(self.get_parameter('status_topic').value),
            self._status_callback,
            status_qos,
        )
        self.create_subscription(
            TerrainStatus,
            str(self.get_parameter('terrain_status_topic').value),
            self._terrain_callback,
            status_qos,
        )
        self.create_subscription(
            SurfaceCondition,
            str(self.get_parameter('surface_topic').value),
            self._surface_callback,
            qos_profile_sensor_data,
        )

        self._command_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('safe_command_topic').value),
            10,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self._timer = self.create_timer(
            1.0 / self._publish_rate, self._timer_callback
        )

        self.get_logger().info(
            'Safety supervisor ready at %.1f Hz; terrain TOF is %s and '
            'surface perception is %s'
            % (
                self._publish_rate,
                'required' if self._require_ranges else 'optional',
                'required' if self._require_surface else 'optional',
            )
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _command_callback(self, msg: TwistStamped) -> None:
        self._last_command = msg
        self._last_command_time = self._now_seconds()

    def _status_callback(self, msg: WalkerStatus) -> None:
        self._last_status = msg
        self._last_status_time = self._now_seconds()

    def _terrain_callback(self, msg: TerrainStatus) -> None:
        self._last_terrain = msg
        self._last_terrain_time = self._now_seconds()

    def _surface_callback(self, msg: SurfaceCondition) -> None:
        self._last_surface = msg
        self._last_surface_time = self._now_seconds()

    def _range_callback(self, side: str, msg: Range) -> None:
        distance = float(msg.range)
        minimum = max(0.0, float(msg.min_range))
        maximum = float(msg.max_range)
        valid_limits = math.isfinite(maximum) and maximum > minimum

        if distance == math.inf:
            if valid_limits:
                distance = maximum
                valid = True
            else:
                distance = math.nan
                valid = False
        else:
            valid = (
                math.isfinite(distance)
                and distance >= minimum
                and valid_limits
                and distance <= maximum
            )

        self._ranges[side] = {
            'distance': distance,
            'valid': valid,
            'time': self._now_seconds(),
        }

    @staticmethod
    def _age(now: float, received_at: Optional[float]) -> float:
        if received_at is None:
            return math.inf
        if now < received_at:
            return math.inf
        return now - received_at

    def _status_reasons(self, now: float) -> List[str]:
        if self._last_status is None:
            return ['status_missing']
        if self._age(now, self._last_status_time) > self._status_timeout:
            return ['status_stale']

        status = self._last_status
        reasons: List[str] = []
        if not status.link_ok:
            reasons.append('link_not_ok')
        if status.state == WalkerStatus.STATE_DISCONNECTED:
            reasons.append('disconnected')
        if status.state == WalkerStatus.STATE_SAFE_STOP:
            reasons.append('safe_stop')
        if status.estop or status.state == WalkerStatus.STATE_ESTOP:
            reasons.append('estop')
        if status.watchdog_timeout:
            reasons.append('mcu_watchdog_timeout')
        if status.fault_bits != WalkerStatus.FAULT_NONE:
            reasons.append('mcu_fault')
        if status.state == WalkerStatus.STATE_FAULT:
            reasons.append('fault_state')
        if (
            not status.armed
            or status.state == WalkerStatus.STATE_DISARMED
        ):
            reasons.append('disarmed')
        if status.state not in (
            WalkerStatus.STATE_ARMED,
            WalkerStatus.STATE_DISCONNECTED,
            WalkerStatus.STATE_DISARMED,
            WalkerStatus.STATE_SAFE_STOP,
            WalkerStatus.STATE_ESTOP,
            WalkerStatus.STATE_FAULT,
        ):
            reasons.append('invalid_state')
        if self._require_deadman and not status.deadman:
            reasons.append('deadman_released')

        telemetry_age = float(status.telemetry_age)
        if (
            not math.isfinite(telemetry_age)
            or telemetry_age < 0.0
            or telemetry_age > self._max_telemetry_age
        ):
            reasons.append('telemetry_stale')
        return reasons

    def _command_reasons(self, now: float) -> List[str]:
        if self._last_command is None:
            return ['command_missing']
        if self._age(now, self._last_command_time) > self._command_timeout:
            return ['command_stale']
        linear = float(self._last_command.twist.linear.x)
        angular = float(self._last_command.twist.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            return ['command_nonfinite']
        if abs(angular) > self._max_angular:
            return ['angular_command_unsupported']
        return []

    def _terrain_reasons(self, now: float) -> List[str]:
        if self._last_terrain is None:
            return ['terrain_status_missing'] if self._require_ranges else []
        if self._age(now, self._last_terrain_time) > self._range_timeout:
            return ['terrain_status_stale'] if self._require_ranges else []

        terrain = self._last_terrain
        reasons: List[str] = []
        telemetry_age = float(terrain.telemetry_age)
        if (
            not math.isfinite(telemetry_age)
            or telemetry_age < 0.0
            or telemetry_age > self._max_telemetry_age
        ):
            reasons.append('terrain_telemetry_stale')
        if (
            not terrain.tof_valid
            or terrain.tof_alert == TerrainStatus.TOF_INVALID
        ):
            reasons.append('terrain_tof_invalid')
        elif terrain.tof_alert == TerrainStatus.TOF_RAISED:
            reasons.append('terrain_raised_obstacle')
        elif terrain.tof_alert == TerrainStatus.TOF_DROP:
            reasons.append('terrain_drop')
        elif terrain.tof_alert not in (
            TerrainStatus.TOF_NORMAL,
            TerrainStatus.TOF_CANDIDATE_RAISED,
            TerrainStatus.TOF_CANDIDATE_DROP,
        ):
            reasons.append('terrain_alert_invalid')
        if terrain.fault_bits & TerrainStatus.FAULT_TOF_INVALID:
            reasons.append('terrain_tof_fault')
        return reasons

    def _range_scale(self, distance: float) -> float:
        if distance <= self._stop_distance:
            return 0.0
        if distance >= self._slow_distance:
            return 1.0
        return (
            (distance - self._stop_distance)
            / (self._slow_distance - self._stop_distance)
        )

    def _range_state(
        self, now: float
    ) -> Tuple[List[str], Dict[str, float], Dict[str, float]]:
        reasons: List[str] = []
        distances = {'left': math.nan, 'right': math.nan}
        scales = {'left': 1.0, 'right': 1.0}

        for side in ('left', 'right'):
            sample = self._ranges[side]
            sample_time = sample['time']
            fresh = (
                sample_time is not None
                and self._age(now, sample_time) <= self._range_timeout
            )
            valid = bool(sample['valid'])

            if not fresh or not valid:
                # Front-facing ranges are optional. require_range_sensors now
                # refers to the installed downward TOF TerrainStatus stream.
                continue

            distance = float(sample['distance'])
            distances[side] = distance
            scales[side] = self._range_scale(distance)

        return reasons, distances, scales

    def _surface_state(
        self, now: float
    ) -> Tuple[List[str], float, int, float]:
        if self._last_surface is None:
            reasons = ['surface_missing'] if self._require_surface else []
            return reasons, 1.0, SurfaceCondition.UNKNOWN, math.nan
        if self._age(now, self._last_surface_time) > self._surface_timeout:
            reasons = ['surface_stale'] if self._require_surface else []
            return reasons, 1.0, SurfaceCondition.UNKNOWN, math.nan

        surface = self._last_surface
        confidence = float(surface.confidence)
        scale = float(surface.recommended_speed_scale)
        known_classifications = {
            SurfaceCondition.SMOOTH,
            SurfaceCondition.ROUGH,
            SurfaceCondition.WET,
            SurfaceCondition.GRAVEL,
            SurfaceCondition.STEP,
            SurfaceCondition.HOLE,
        }
        valid = (
            bool(surface.valid)
            and surface.classification in known_classifications
            and math.isfinite(confidence)
            and 0.0 <= confidence <= 1.0
            and math.isfinite(scale)
            and 0.0 <= scale <= self._max_surface_scale
        )
        if not valid:
            reasons = ['surface_invalid'] if self._require_surface else []
            return reasons, 1.0, surface.classification, confidence
        return [], scale, surface.classification, confidence

    @staticmethod
    def _slew(
        current: float,
        target: float,
        acceleration: float,
        deceleration: float,
        dt: float,
    ) -> float:
        gaining_speed = (
            current == 0.0
            or (current * target > 0.0 and abs(target) > abs(current))
        )
        rate = acceleration if gaining_speed else deceleration
        max_change = max(0.0, rate) * max(0.0, dt)
        return current + _clamp(target - current, -max_change, max_change)

    def _desired_command(
        self,
        range_scales: Dict[str, float],
        surface_scale: float,
    ) -> Tuple[float, float, List[str]]:
        assert self._last_command is not None
        requested_linear = float(self._last_command.twist.linear.x)
        requested_angular = float(self._last_command.twist.angular.z)

        finite_input = (
            math.isfinite(requested_linear)
            and math.isfinite(requested_angular)
        )
        if not finite_input:
            return 0.0, 0.0, ['nonfinite_command']
        linear = _clamp(
            requested_linear, -self._max_reverse, self._max_forward
        )
        angular = _clamp(
            requested_angular, -self._max_angular, self._max_angular
        )

        limiting_scale = 1.0
        if linear > 0.0:
            limiting_scale = min(
                range_scales['left'], range_scales['right']
            )
            linear *= limiting_scale

        turn_scale = 1.0
        if angular != 0.0:
            # With only two forward-looking sensors, either side can sweep
            # into an obstacle during a turn.  Be conservative until a full
            # footprint-aware collision monitor is available.
            turn_scale = min(
                range_scales['left'], range_scales['right']
            )
            angular *= turn_scale

        linear *= surface_scale
        angular *= surface_scale
        linear = _clamp(linear, -self._max_reverse, self._max_forward)
        angular = _clamp(angular, -self._max_angular, self._max_angular)

        notes: List[str] = []
        active_scale = min(
            limiting_scale,
            turn_scale,
        )
        if active_scale <= 0.0 and (
            requested_linear > 0.0 or requested_angular != 0.0
        ):
            notes.append('obstacle_stop')
        elif active_scale < 1.0:
            notes.append('obstacle_slowdown')
        if surface_scale <= 0.0 and (
            requested_linear != 0.0 or requested_angular != 0.0
        ):
            notes.append('surface_stop')
        elif surface_scale < 1.0:
            notes.append('surface_slowdown')
        elif surface_scale > 1.0:
            notes.append('surface_speedup')

        return linear, angular, notes

    def _timer_callback(self) -> None:
        now = self._now_seconds()
        dt = now - self._last_tick_time
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self._publish_rate
        self._last_tick_time = now

        hard_stop_reasons = self._command_reasons(now)
        hard_stop_reasons.extend(self._status_reasons(now))
        hard_stop_reasons.extend(self._terrain_reasons(now))
        range_reasons, distances, range_scales = self._range_state(now)
        hard_stop_reasons.extend(range_reasons)
        (
            surface_reasons,
            surface_scale,
            surface_classification,
            surface_confidence,
        ) = self._surface_state(now)
        hard_stop_reasons.extend(surface_reasons)
        motion_stop_reasons = [
            reason for reason in hard_stop_reasons
            if reason != 'disarmed'
        ]

        operating_notes: List[str] = []
        if motion_stop_reasons:
            # Stale or unsafe state must never coast on the last valid command.
            self._output_linear = 0.0
            self._output_angular = 0.0
        else:
            desired_linear, desired_angular, operating_notes = (
                self._desired_command(range_scales, surface_scale)
            )
            if (
                'obstacle_stop' in operating_notes
                or 'surface_stop' in operating_notes
            ):
                # Hazard entry is immediate; a valid clear result is required
                # before motion can resume.
                self._output_linear = 0.0
                self._output_angular = 0.0
            else:
                self._output_linear = self._slew(
                    self._output_linear,
                    desired_linear,
                    self._linear_accel,
                    self._linear_decel,
                    dt,
                )
                self._output_angular = self._slew(
                    self._output_angular,
                    desired_angular,
                    self._angular_accel,
                    self._angular_decel,
                    dt,
                )

        # Keep forwarding a valid supervised command while DISARMED so an
        # explicit enable request can activate the controller. Other invalid
        # or stale inputs publish one immediate zero and then go silent,
        # forcing the bridge timeout to disarm the MCU.
        may_stream_command = (
            not hard_stop_reasons
            or set(hard_stop_reasons).issubset({'disarmed'})
        )
        if may_stream_command or not self._command_output_suppressed:
            output = TwistStamped()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = self._output_frame_id
            output.twist.linear.x = self._output_linear
            output.twist.angular.z = self._output_angular
            self._command_publisher.publish(output)
        self._command_output_suppressed = not may_stream_command

        all_reasons = hard_stop_reasons + operating_notes
        summary = ', '.join(all_reasons) if all_reasons else 'ready'
        if summary != self._last_log_summary:
            if hard_stop_reasons:
                self.get_logger().warning('Motion inhibited: ' + summary)
            elif operating_notes:
                self.get_logger().warning('Motion limited: ' + summary)
            else:
                self.get_logger().info('Motion supervision ready')
            self._last_log_summary = summary

        if now - self._last_diagnostic_time >= 1.0 / self._diagnostic_rate:
            self._publish_diagnostics(
                now,
                hard_stop_reasons,
                operating_notes,
                distances,
                range_scales,
                surface_scale,
                surface_classification,
                surface_confidence,
            )
            self._last_diagnostic_time = now

    def _publish_diagnostics(
        self,
        now: float,
        hard_stop_reasons: List[str],
        operating_notes: List[str],
        distances: Dict[str, float],
        range_scales: Dict[str, float],
        surface_scale: float,
        surface_classification: int,
        surface_confidence: float,
    ) -> None:
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'SafeStride/Safety Supervisor'
        diagnostic.hardware_id = 'safestride'

        severe = {
            'estop',
            'mcu_fault',
            'fault_state',
            'mcu_watchdog_timeout',
            'link_not_ok',
            'disconnected',
            'invalid_state',
            'status_stale',
            'telemetry_stale',
            'command_nonfinite',
            'surface_missing',
            'surface_stale',
            'surface_invalid',
            'terrain_status_missing',
            'terrain_status_stale',
            'terrain_telemetry_stale',
            'terrain_tof_invalid',
            'terrain_raised_obstacle',
            'terrain_drop',
            'terrain_alert_invalid',
            'terrain_tof_fault',
        }
        if any(reason in severe for reason in hard_stop_reasons):
            diagnostic.level = DiagnosticStatus.ERROR
        elif hard_stop_reasons or operating_notes:
            diagnostic.level = DiagnosticStatus.WARN
        else:
            diagnostic.level = DiagnosticStatus.OK

        all_reasons = hard_stop_reasons + operating_notes
        diagnostic.message = ', '.join(all_reasons) if all_reasons else 'ready'
        status = self._last_status
        diagnostic.values = [
            KeyValue(
                key='motion_inhibited',
                value=_bool_text(bool(hard_stop_reasons)),
            ),
            KeyValue(
                key='command_output_suppressed',
                value=_bool_text(self._command_output_suppressed),
            ),
            KeyValue(
                key='command_age_s',
                value='%.3f' % self._age(now, self._last_command_time),
            ),
            KeyValue(
                key='status_age_s',
                value='%.3f' % self._age(now, self._last_status_time),
            ),
            KeyValue(
                key='left_range_m',
                value=(
                    'unavailable'
                    if math.isnan(distances['left'])
                    else '%.3f' % distances['left']
                ),
            ),
            KeyValue(
                key='right_range_m',
                value=(
                    'unavailable'
                    if math.isnan(distances['right'])
                    else '%.3f' % distances['right']
                ),
            ),
            KeyValue(
                key='left_range_scale',
                value='%.3f' % range_scales['left'],
            ),
            KeyValue(
                key='right_range_scale',
                value='%.3f' % range_scales['right'],
            ),
            KeyValue(
                key='surface_required',
                value=_bool_text(self._require_surface),
            ),
            KeyValue(
                key='surface_age_s',
                value='%.3f' % self._age(now, self._last_surface_time),
            ),
            KeyValue(
                key='surface_classification',
                value=str(surface_classification),
            ),
            KeyValue(
                key='surface_confidence',
                value=(
                    'unavailable'
                    if not math.isfinite(surface_confidence)
                    else '%.3f' % surface_confidence
                ),
            ),
            KeyValue(
                key='surface_speed_scale',
                value='%.3f' % surface_scale,
            ),
            KeyValue(
                key='output_linear_mps',
                value='%.3f' % self._output_linear,
            ),
            KeyValue(
                key='output_angular_radps',
                value='%.3f' % self._output_angular,
            ),
            KeyValue(
                key='fault_bits',
                value='0x%04x' % (status.fault_bits if status else 0),
            ),
            KeyValue(
                key='mcu_state',
                value=str(status.state if status else -1),
            ),
            KeyValue(
                key='link_ok',
                value=_bool_text(bool(status and status.link_ok)),
            ),
            KeyValue(
                key='armed',
                value=_bool_text(bool(status and status.armed)),
            ),
            KeyValue(
                key='deadman',
                value=_bool_text(bool(status and status.deadman)),
            ),
            KeyValue(
                key='estop',
                value=_bool_text(bool(status and status.estop)),
            ),
            KeyValue(
                key='watchdog_timeout',
                value=_bool_text(bool(status and status.watchdog_timeout)),
            ),
            KeyValue(
                key='telemetry_age_s',
                value=(
                    'unavailable'
                    if status is None
                    else '%.3f' % status.telemetry_age
                ),
            ),
            KeyValue(
                key='boot_id',
                value=str(status.boot_id if status else 0),
            ),
            KeyValue(
                key='session_id',
                value=str(status.session_id if status else 0),
            ),
            KeyValue(
                key='last_applied_command_sequence',
                value=str(
                    status.last_applied_command_sequence if status else 0
                ),
            ),
            KeyValue(
                key='crc_error_count',
                value=str(status.crc_error_count if status else 0),
            ),
            KeyValue(
                key='frame_error_count',
                value=str(status.frame_error_count if status else 0),
            ),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self._diagnostic_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a final best-effort zero before tearing down the publisher.
        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.header.frame_id = node._output_frame_id
        node._command_publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
