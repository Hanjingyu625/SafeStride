"""Short-lived compass bearing anchored by GPS and propagated by IMU yaw rate.

An IMU never supplies the initial north reference. Positive ROS yaw is
counterclockwise; compass bearing increases clockwise.
"""

import math


class HeadingFusion:
    def __init__(self, gps_timeout_s=5.0, imu_timeout_s=0.35, coast_s=20.0):
        self.gps_timeout_s = gps_timeout_s
        self.imu_timeout_s = imu_timeout_s
        self.coast_s = coast_s
        self.angle = None
        self.anchor_time = None
        self.imu_time = None
        self.last_gps_time = None
        self.last_turn = -math.inf
        self.bias = 0.0
        self.quiet_since = None
        self.source = 'unavailable'
        self.confidence = 0.0

    def invalidate_imu(self):
        # A gap can hide a turn. Require a new GPS observation after the gap.
        self.angle = self.anchor_time = self.imu_time = None
        self.quiet_since = None

    def gyro(self, yaw_rate, now, stationary=False):
        if not math.isfinite(yaw_rate) or not math.isfinite(now):
            self.invalidate_imu()
            return
        dt = None if self.imu_time is None else now - self.imu_time
        if dt is not None and not 0 < dt <= self.imu_timeout_s:
            self.invalidate_imu()
            dt = None
        self.imu_time = now
        # Wheel standstill alone cannot prove no pivot: also require quiet gyro.
        if stationary and abs(yaw_rate) < math.radians(0.5):
            if self.quiet_since is None:
                self.quiet_since = now
            if now - self.quiet_since >= 2.0 and dt is not None:
                self.bias += min(dt / 10.0, 1.0) * (yaw_rate - self.bias)
        else:
            self.quiet_since = None
        corrected = yaw_rate - self.bias
        if abs(corrected) > math.radians(3.0):
            self.last_turn = now
        if dt is not None and self.angle is not None:
            self.angle = (self.angle - math.degrees(corrected * dt)) % 360.0

    def gps(self, bearing, observed_at, now):
        if (bearing is None or observed_at is None
                or not math.isfinite(bearing)
                or not 0 <= now - observed_at <= self.gps_timeout_s
                or (self.last_gps_time is not None and observed_at <= self.last_gps_time)):
            return
        self.last_gps_time = observed_at
        # Course/position deltas during a turn can describe the old trajectory.
        if now - self.last_turn < 1.0:
            return
        if self.angle is None or self.anchor_time is None or now - self.anchor_time > self.coast_s:
            self.angle = bearing % 360.0
        else:
            error = (bearing - self.angle + 180.0) % 360.0 - 180.0
            self.angle = (self.angle + 0.2 * error) % 360.0
        self.anchor_time = observed_at

    def heading(self, now):
        self.source = 'unavailable'
        self.confidence = 0.0
        if self.angle is None or self.anchor_time is None:
            return None
        age = now - self.anchor_time
        imu_fresh = self.imu_time is not None and -1e-6 <= now - self.imu_time <= self.imu_timeout_s
        if self.imu_time is not None and not imu_fresh:
            self.invalidate_imu()
            return None
        limit = self.coast_s if imu_fresh else self.gps_timeout_s
        if not 0 <= age <= limit:
            return None
        self.source = 'gps_gyro' if imu_fresh else 'gps'
        self.confidence = max(0.0, 1.0 - age / limit)
        return self.angle
