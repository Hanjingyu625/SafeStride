"""ROS-independent safety primitives used by the command supervisor."""

import math
from typing import Optional, Tuple


def finite_parameter(
    name: str,
    value,
    *,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
    minimum_inclusive: bool = True,
) -> float:
    """Return a finite safety parameter or raise instead of coercing it."""

    if isinstance(value, bool):
        raise ValueError(f'{name} must be a number, not bool')
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} must be a number') from error
    if not math.isfinite(parsed):
        raise ValueError(f'{name} must be finite')
    if minimum is not None:
        below = (
            parsed < minimum
            if minimum_inclusive
            else parsed <= minimum
        )
        if below:
            operator = '>=' if minimum_inclusive else '>'
            raise ValueError(f'{name} must be {operator} {minimum}')
    if maximum is not None and parsed > maximum:
        raise ValueError(f'{name} must be <= {maximum}')
    return parsed


def combine_speed_scales(
    surface_scale: float,
    slope_scale: float,
    maximum_scale: float,
) -> float:
    """Combine speed modifiers without letting an assist cancel a slowdown."""

    if surface_scale < 1.0 or slope_scale < 1.0:
        return min(surface_scale, slope_scale)
    return min(surface_scale * slope_scale, maximum_scale)


class SlopeSpeedPolicy:
    """Confirm and hysteretically classify pitch as level/uphill/downhill."""

    DOWNHILL = 'downhill'
    LEVEL = 'level'
    UPHILL = 'uphill'

    def __init__(
        self,
        *,
        enter_angle_rad: float,
        exit_angle_rad: float,
        confirmation_time_s: float,
        uphill_pitch_sign: float,
        pitch_offset_rad: float,
        downhill_speed_scale: float,
        uphill_speed_scale: float,
    ) -> None:
        if exit_angle_rad >= enter_angle_rad:
            raise ValueError('slope exit angle must be below enter angle')
        if uphill_pitch_sign not in (-1.0, 1.0):
            raise ValueError('uphill_pitch_sign must be +1.0 or -1.0')
        self._enter_angle = enter_angle_rad
        self._exit_angle = exit_angle_rad
        self._confirmation_time = confirmation_time_s
        self._uphill_pitch_sign = uphill_pitch_sign
        self._pitch_offset = pitch_offset_rad
        self._downhill_scale = downhill_speed_scale
        self._uphill_scale = uphill_speed_scale
        self._state = self.LEVEL
        self._candidate = self.LEVEL
        self._candidate_since: Optional[float] = None

    def reset(self) -> None:
        self._state = self.LEVEL
        self._candidate = self.LEVEL
        self._candidate_since = None

    def _desired_state(self, normalized_pitch: float) -> str:
        if self._state == self.UPHILL:
            if normalized_pitch <= -self._enter_angle:
                return self.DOWNHILL
            if normalized_pitch <= self._exit_angle:
                return self.LEVEL
            return self.UPHILL
        if self._state == self.DOWNHILL:
            if normalized_pitch >= self._enter_angle:
                return self.UPHILL
            if normalized_pitch >= -self._exit_angle:
                return self.LEVEL
            return self.DOWNHILL
        if normalized_pitch >= self._enter_angle:
            return self.UPHILL
        if normalized_pitch <= -self._enter_angle:
            return self.DOWNHILL
        return self.LEVEL

    def update(
        self,
        *,
        pitch_rad: float,
        sample_valid: bool,
        now_s: float,
    ) -> Tuple[float, str, float]:
        """Return speed scale, confirmed state, and polarity-adjusted pitch."""

        if (
            not sample_valid
            or not math.isfinite(pitch_rad)
            or not math.isfinite(now_s)
        ):
            self.reset()
            return 1.0, self.LEVEL, math.nan

        normalized_pitch = (
            pitch_rad - self._pitch_offset
        ) * self._uphill_pitch_sign
        desired = self._desired_state(normalized_pitch)
        if desired == self._state:
            self._candidate = self._state
            self._candidate_since = None
        elif desired != self._candidate:
            self._candidate = desired
            self._candidate_since = now_s
        elif (
            self._candidate_since is not None
            and now_s - self._candidate_since >= self._confirmation_time
        ):
            self._state = desired
            self._candidate = desired
            self._candidate_since = None

        scale = 1.0
        if self._state == self.UPHILL:
            scale = self._uphill_scale
        elif self._state == self.DOWNHILL:
            scale = self._downhill_scale
        return scale, self._state, normalized_pitch


__all__ = [
    'SlopeSpeedPolicy',
    'combine_speed_scales',
    'finite_parameter',
]
