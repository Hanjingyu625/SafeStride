"""Testable GPS movement, heading, and stuck-coordinate tracking."""

import math
from typing import Optional, Tuple

from .crosswalk_data import bearing_deg, haversine_m


def select_motion_measurement(
    *,
    odom_fresh: bool,
    odom_speed_mps: Optional[float],
    wheel_motion_active: bool,
    gps_fresh: bool,
    gps_speed_mps: Optional[float],
    allow_gps_speed_fallback: bool,
) -> Tuple[Optional[float], str, bool]:
    """Select a motion source without treating GNSS drift as wheel motion."""
    if odom_fresh:
        return odom_speed_mps, 'wheel_odom', wheel_motion_active
    if allow_gps_speed_fallback and gps_fresh:
        return (
            gps_speed_mps,
            'gps_filtered',
            bool(gps_speed_mps is not None and gps_speed_mps > 0.0),
        )
    return None, 'unavailable', False


class GpsMotionTracker:
    def __init__(
        self,
        *,
        change_threshold_m: float,
        heading_min_move_m: float,
        heading_max_step_m: float,
    ) -> None:
        values = (
            change_threshold_m,
            heading_min_move_m,
            heading_max_step_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError(
                'GPS motion distances must be finite and positive'
            )
        if heading_max_step_m < heading_min_move_m:
            raise ValueError('heading_max_step_m must exceed minimum movement')
        self._change_threshold_m = change_threshold_m
        self._heading_min_move_m = heading_min_move_m
        self._heading_max_step_m = heading_max_step_m
        self._change_anchor: Optional[Tuple[float, float]] = None
        self._heading_anchor: Optional[Tuple[float, float]] = None
        self._position_change_time: Optional[float] = None
        self._heading: Optional[float] = None
        self._heading_time: Optional[float] = None
        self.heading_source = 'unavailable'

    def update(
        self,
        latitude: float,
        longitude: float,
        now: float,
        *,
        allow_heading: bool = True,
    ) -> None:
        current = (latitude, longitude)
        if self._change_anchor is None:
            self._change_anchor = current
            self._position_change_time = now
        else:
            changed_distance = haversine_m(
                self._change_anchor[0],
                self._change_anchor[1],
                latitude,
                longitude,
            )
            if changed_distance >= self._change_threshold_m:
                self._change_anchor = current
                self._position_change_time = now

        if not allow_heading:
            self._heading_anchor = current
            return
        if self._heading_anchor is None:
            self._heading_anchor = current
            return
        heading_distance = haversine_m(
            self._heading_anchor[0],
            self._heading_anchor[1],
            latitude,
            longitude,
        )
        if heading_distance < self._heading_min_move_m:
            return
        if heading_distance <= self._heading_max_step_m:
            self._heading = bearing_deg(
                self._heading_anchor[0],
                self._heading_anchor[1],
                latitude,
                longitude,
            )
            self._heading_time = now
            self.heading_source = 'position_delta'
        self._heading_anchor = current

    def set_course(self, course_deg: float, now: float) -> None:
        if not math.isfinite(course_deg):
            raise ValueError('course must be finite')
        self._heading = course_deg % 360.0
        self._heading_time = now
        self.heading_source = 'rmc_course'

    def heading(self, now: float, timeout_s: float) -> Optional[float]:
        if (
            self._heading_time is not None
            and now >= self._heading_time
            and now - self._heading_time <= timeout_s
        ):
            return self._heading
        return None

    def coordinates_stuck(
        self,
        now: float,
        speed_mps: Optional[float],
        *,
        timeout_s: float,
        minimum_speed_mps: float,
    ) -> bool:
        if speed_mps is None or speed_mps < minimum_speed_mps:
            return False
        return (
            self._position_change_time is None
            or now < self._position_change_time
            or now - self._position_change_time > timeout_s
        )


__all__ = ['GpsMotionTracker', 'select_motion_measurement']
