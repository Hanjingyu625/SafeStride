"""Publish a user-responsive straight-line command before safety limiting."""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from safestride_interfaces.msg import DriveCommand, TerrainStatus, WalkerStatus

from .safety_logic import finite_parameter


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class CruiseCommandNode(Node):
    """Adapt the cruise request when Hall, IMU and speed error show intent."""

    def __init__(self) -> None:
        super().__init__('cruise_command')
        defaults = {
            'speed_mps': 1.0,
            'minimum_speed_mps': 0.0,
            'maximum_speed_mps': 1.15,
            'push_speed_step_mps': 0.03,
            'pull_speed_step_mps': 0.05,
            'push_repeat_s': 0.40,
            'pull_repeat_s': 0.30,
            'hall_delta_threshold_mps': 0.05,
            'imu_delta_velocity_threshold_mps': 0.03,
            'feedback_threshold_pwm': 3.0,
            'intent_window_s': 0.50,
            'target_proximity_mps': 0.05,
            'maximum_learning_pitch_deg': 3.0,
            'status_timeout_s': 0.50,
            'hall_speed_max_age_s': 0.75,
            'imu_timeout_s': 0.30,
            'terrain_timeout_s': 0.35,
            'drive_command_timeout_s': 0.50,
            'gps_timeout_s': 2.0,
            'gps_disagreement_mps': 0.40,
            'gps_disagreement_hold_s': 2.0,
            'imu_forward_sign': 1.0,
            'publish_rate_hz': 20.0,
            'status_topic': '/walker/status',
            'imu_topic': '/terrain/imu',
            'terrain_topic': '/terrain/status',
            'gps_speed_topic': '/gps/speed',
            'drive_command_topic': '/drive/command',
            'command_topic': '/cmd_vel',
            'frame_id': 'base_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._minimum_speed = self._number(
            'minimum_speed_mps', minimum=0.0, maximum=1.15)
        self._maximum_speed = self._number(
            'maximum_speed_mps', minimum=self._minimum_speed, maximum=1.15)
        self._default_speed = self._number(
            'speed_mps',
            minimum=self._minimum_speed,
            maximum=self._maximum_speed,
        )
        self._push_step = self._positive('push_speed_step_mps')
        self._pull_step = self._positive('pull_speed_step_mps')
        self._push_repeat = self._positive('push_repeat_s')
        self._pull_repeat = self._positive('pull_repeat_s')
        self._hall_delta_threshold = self._positive(
            'hall_delta_threshold_mps')
        self._imu_delta_velocity_threshold = self._positive(
            'imu_delta_velocity_threshold_mps')
        self._feedback_threshold = self._positive('feedback_threshold_pwm')
        self._intent_window = self._positive('intent_window_s')
        self._target_proximity = self._positive('target_proximity_mps')
        self._maximum_pitch = math.radians(self._number(
            'maximum_learning_pitch_deg', minimum=0.0, maximum=45.0))
        self._status_timeout = self._positive('status_timeout_s')
        self._hall_speed_max_age = self._positive('hall_speed_max_age_s')
        self._imu_timeout = self._positive('imu_timeout_s')
        self._terrain_timeout = self._positive('terrain_timeout_s')
        self._drive_timeout = self._positive('drive_command_timeout_s')
        self._gps_timeout = self._positive('gps_timeout_s')
        self._gps_disagreement = self._positive('gps_disagreement_mps')
        self._gps_disagreement_hold = self._positive(
            'gps_disagreement_hold_s')
        self._imu_forward_sign = self._number(
            'imu_forward_sign', minimum=-1.0, maximum=1.0)
        if self._imu_forward_sign not in (-1.0, 1.0):
            raise ValueError('imu_forward_sign must be -1.0 or 1.0')
        publish_rate = self._positive('publish_rate_hz', maximum=100.0)

        self._frame_id = self._topic('frame_id')
        command_topic = self._topic('command_topic')
        self._target_speed = self._default_speed
        self._status = None
        self._status_time = None
        self._terrain = None
        self._terrain_time = None
        self._drive_command = None
        self._drive_command_time = None
        self._imu_time = None
        self._gps_speed = None
        self._gps_time = None
        self._gps_disagreement_since = None
        self._last_deadman = False
        self._next_push_time = 0.0
        self._next_pull_time = 0.0
        self._imu_bias_m_s2 = 0.0
        self._imu_bias_ready = False
        self._speed_history = deque()
        self._imu_history = deque()

        self._publisher = self.create_publisher(
            TwistStamped, command_topic, 10)
        self.create_subscription(
            WalkerStatus,
            self._topic('status_topic'),
            self._status_callback,
            10,
        )
        self.create_subscription(
            Imu,
            self._topic('imu_topic'),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TerrainStatus,
            self._topic('terrain_topic'),
            self._terrain_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self._topic('gps_speed_topic'),
            self._gps_speed_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DriveCommand,
            self._topic('drive_command_topic'),
            self._drive_command_callback,
            10,
        )
        self._timer = self.create_timer(
            1.0 / publish_rate, self._publish_command)
        self.get_logger().info(
            'adaptive cruise ready at %.3f m/s, range %.3f..%.3f m/s'
            % (self._target_speed, self._minimum_speed, self._maximum_speed)
        )

    def _number(
        self,
        name: str,
        *,
        minimum: float,
        maximum: float,
    ) -> float:
        return finite_parameter(
            name,
            self.get_parameter(name).value,
            minimum=minimum,
            maximum=maximum,
        )

    def _positive(self, name: str, *, maximum: float = 60.0) -> float:
        return finite_parameter(
            name,
            self.get_parameter(name).value,
            minimum=0.0,
            maximum=maximum,
            minimum_inclusive=False,
        )

    def _topic(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError('%s must not be empty' % name)
        return value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _append_sample(self, history, now: float, value: float) -> None:
        history.append((now, value))
        cutoff = now - max(2.0, 3.0 * self._intent_window)
        while history and history[0][0] < cutoff:
            history.popleft()

    def _window_delta(self, history, now: float) -> float:
        cutoff = now - self._intent_window
        samples = [sample for sample in history if sample[0] >= cutoff]
        if len(samples) < 2 or samples[-1][0] - samples[0][0] < 0.20:
            return 0.0
        return samples[-1][1] - samples[0][1]

    def _imu_delta_velocity(self, now: float) -> float:
        cutoff = now - self._intent_window
        samples = [
            sample for sample in self._imu_history
            if sample[0] >= cutoff
        ]
        if len(samples) < 2 or samples[-1][0] - samples[0][0] < 0.20:
            return 0.0
        delta_velocity = 0.0
        for previous, current in zip(samples, samples[1:]):
            dt = current[0] - previous[0]
            if 0.0 < dt <= 0.20:
                delta_velocity += 0.5 * (previous[1] + current[1]) * dt
        return delta_velocity

    def _status_callback(self, message: WalkerStatus) -> None:
        now = self._now()
        deadman = bool(message.deadman)
        if self._last_deadman and not deadman:
            self._target_speed = self._default_speed
            self._speed_history.clear()
            self._imu_history.clear()
            self._next_push_time = now + self._push_repeat
            self._next_pull_time = now + self._pull_repeat
            self.get_logger().info(
                'hands released; adaptive cruise reset to %.3f m/s'
                % self._target_speed)
        self._last_deadman = deadman
        self._status = message
        self._status_time = now
        speed = float(message.measured_speed_m_s)
        if bool(message.speed_valid) and math.isfinite(speed) and speed >= 0.0:
            self._append_sample(self._speed_history, now, speed)

    def _imu_callback(self, message: Imu) -> None:
        now = self._now()
        acceleration = (
            float(message.linear_acceleration.x) * self._imu_forward_sign)
        if (
            message.linear_acceleration_covariance[0] < 0.0
            or not math.isfinite(acceleration)
        ):
            return
        status = self._status
        stationary = (
            status is not None
            and self._fresh(now, self._status_time, self._status_timeout)
            and not bool(status.deadman)
            and math.isfinite(float(status.measured_speed_m_s))
            and abs(float(status.measured_speed_m_s)) < 0.03
        )
        if stationary:
            alpha = 0.02 if self._imu_bias_ready else 1.0
            self._imu_bias_m_s2 += alpha * (
                acceleration - self._imu_bias_m_s2)
            self._imu_bias_ready = True
        corrected = acceleration - self._imu_bias_m_s2
        self._append_sample(self._imu_history, now, corrected)
        self._imu_time = now

    def _terrain_callback(self, message: TerrainStatus) -> None:
        self._terrain = message
        self._terrain_time = self._now()

    def _gps_speed_callback(self, message: Float32) -> None:
        speed = float(message.data)
        if math.isfinite(speed) and 0.0 <= speed <= 3.0:
            self._gps_speed = speed
            self._gps_time = self._now()

    def _drive_command_callback(self, message: DriveCommand) -> None:
        self._drive_command = message
        self._drive_command_time = self._now()

    def _fresh(self, now: float, stamp, timeout: float) -> bool:
        return stamp is not None and 0.0 <= now - stamp <= timeout

    def _gps_blocks_adaptation(
        self,
        now: float,
        speed: float,
        hall_delta: float,
    ) -> bool:
        if not self._fresh(now, self._gps_time, self._gps_timeout):
            self._gps_disagreement_since = None
            return False
        disagreement = abs(float(self._gps_speed) - speed)
        # GPS is a slow sanity check, never a prerequisite for prompt intent.
        # Only block after both the Hall speed and disagreement stay settled.
        if (
            disagreement > self._gps_disagreement
            and abs(hall_delta) < self._hall_delta_threshold
        ):
            if self._gps_disagreement_since is None:
                self._gps_disagreement_since = now
            return (
                now - self._gps_disagreement_since
                >= self._gps_disagreement_hold
            )
        self._gps_disagreement_since = None
        return False

    def _adaptation_allowed(self, now: float) -> bool:
        status = self._status
        terrain = self._terrain
        drive = self._drive_command
        if (
            status is None
            or terrain is None
            or drive is None
            or not self._fresh(now, self._status_time, self._status_timeout)
            or not self._fresh(now, self._imu_time, self._imu_timeout)
            or not self._fresh(now, self._terrain_time, self._terrain_timeout)
            or not self._fresh(
                now, self._drive_command_time, self._drive_timeout)
        ):
            return False
        speed = float(status.measured_speed_m_s)
        speed_age = float(status.speed_age)
        pitch = float(terrain.pitch_rad)
        return (
            bool(status.link_ok)
            and bool(status.armed)
            and bool(status.deadman)
            and not bool(status.braking)
            and int(status.fault_bits) == 0
            and bool(status.speed_valid)
            and math.isfinite(speed)
            and speed >= 0.0
            and math.isfinite(speed_age)
            and 0.0 <= speed_age <= self._hall_speed_max_age
            and bool(terrain.mpu_valid)
            and math.isfinite(pitch)
            and abs(pitch) <= self._maximum_pitch
            and int(drive.mode) == int(DriveCommand.DRIVE)
            and int(drive.slope_ff_pwm) == 0
        )

    def _update_target(self, now: float) -> None:
        if not self._adaptation_allowed(now):
            # Do not carry a slope, braking or stale-sensor transient into the
            # next flat-road intent window.
            self._speed_history.clear()
            self._imu_history.clear()
            self._gps_disagreement_since = None
            return
        status = self._status
        speed = float(status.measured_speed_m_s)
        hall_delta = self._window_delta(self._speed_history, now)
        imu_delta_velocity = self._imu_delta_velocity(now)
        if self._gps_blocks_adaptation(now, speed, hall_delta):
            return

        feedback = float(status.feedback_pwm)
        push_votes = sum((
            hall_delta >= self._hall_delta_threshold,
            imu_delta_velocity >= self._imu_delta_velocity_threshold,
            feedback <= -self._feedback_threshold,
        ))
        pull_votes = sum((
            hall_delta <= -self._hall_delta_threshold,
            imu_delta_velocity <= -self._imu_delta_velocity_threshold,
            feedback >= self._feedback_threshold,
        ))

        # Pull has priority so a user resisting the walker gets the faster
        # response. The vote avoids requiring every noisy sensor to agree.
        if pull_votes >= 2 and now >= self._next_pull_time:
            previous = self._target_speed
            self._target_speed = _clamp(
                previous - self._pull_step,
                self._minimum_speed,
                self._maximum_speed,
            )
            self._next_pull_time = now + self._pull_repeat
            self._next_push_time = max(
                self._next_push_time, now + self._push_repeat)
            if self._target_speed != previous:
                self.get_logger().info(
                    'pull intent %d/3: cruise %.3f -> %.3f m/s'
                    % (pull_votes, previous, self._target_speed))
            return

        near_target = speed >= self._target_speed - self._target_proximity
        if (
            near_target
            and push_votes >= 2
            and now >= self._next_push_time
        ):
            previous = self._target_speed
            self._target_speed = _clamp(
                previous + self._push_step,
                self._minimum_speed,
                self._maximum_speed,
            )
            self._next_push_time = now + self._push_repeat
            self._next_pull_time = max(
                self._next_pull_time, now + self._pull_repeat)
            if self._target_speed != previous:
                self.get_logger().info(
                    'push intent %d/3: cruise %.3f -> %.3f m/s'
                    % (push_votes, previous, self._target_speed))

    def _publish_command(self) -> None:
        now = self._now()
        self._update_target(now)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.twist.linear.x = self._target_speed
        message.twist.angular.z = 0.0
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CruiseCommandNode()
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
