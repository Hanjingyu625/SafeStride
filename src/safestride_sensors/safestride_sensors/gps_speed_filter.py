"""Robust GPS-only speed fallback for low-speed walking."""

import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional


EARTH_RADIUS_M = 6_371_008.8


def _distance_m(
    latitude_a: float,
    longitude_a: float,
    latitude_b: float,
    longitude_b: float,
) -> float:
    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)
    latitude_delta = latitude_b_rad - latitude_a_rad
    longitude_delta = math.radians(longitude_b - longitude_a)
    haversine = (
        math.sin(latitude_delta / 2.0) ** 2
        + math.cos(latitude_a_rad)
        * math.cos(latitude_b_rad)
        * math.sin(longitude_delta / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(
        min(1.0, math.sqrt(haversine))
    )


def _course_coherence(courses_deg) -> float:
    values = [
        math.radians(float(course) % 360.0)
        for course in courses_deg
        if course is not None and math.isfinite(float(course))
    ]
    if len(values) < 3:
        return math.nan
    mean_sine = sum(math.sin(value) for value in values) / len(values)
    mean_cosine = sum(math.cos(value) for value in values) / len(values)
    return math.hypot(mean_sine, mean_cosine)


@dataclass(frozen=True)
class GpsSpeedEstimate:
    raw_speed_mps: float
    filtered_speed_mps: float
    moving: bool
    state: str
    sample_count: int
    window_span_s: float
    displacement_m: float
    path_efficiency: float
    course_coherence: float
    raw_speed_median_mps: float
    position_speed_mps: float
    speed_agreement: float
    movement_threshold_m: float
    quality_ok: bool


@dataclass(frozen=True)
class _Sample:
    time_s: float
    latitude: float
    longitude: float
    speed_mps: float
    course_deg: Optional[float]
    hdop: Optional[float]
    satellites: Optional[int]


class GpsSpeedFilter:
    """Reject stationary GNSS drift while retaining sustained slow motion.

    A single RMC speed sample is not sufficient evidence of motion. The filter
    requires spatially consistent movement across a time window and scales the
    required displacement with HDOP. Wheel odometry remains the authoritative
    speed source elsewhere in the stack; this is the GPS-only fallback.
    """

    def __init__(
        self,
        *,
        window_s: float = 8.0,
        settling_time_s: float = 15.0,
        minimum_span_s: float = 4.0,
        minimum_samples: int = 5,
        minimum_displacement_m: float = 1.0,
        hdop_displacement_scale_m: float = 0.50,
        minimum_path_efficiency: float = 0.55,
        minimum_course_coherence: float = 0.55,
        minimum_speed_agreement: float = 0.60,
        maximum_hdop: float = 5.0,
        minimum_satellites: int = 5,
        require_quality: bool = True,
        enter_confirmations: int = 2,
        exit_confirmations: int = 3,
        smoothing_alpha: float = 0.35,
        maximum_speed_mps: float = 3.0,
    ) -> None:
        positive_values = (
            window_s,
            minimum_span_s,
            minimum_displacement_m,
            hdop_displacement_scale_m,
            maximum_hdop,
            maximum_speed_mps,
        )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in positive_values
        ):
            raise ValueError(
                'GPS speed filter distances and times must be positive'
            )
        if minimum_span_s >= window_s:
            raise ValueError('minimum_span_s must be smaller than window_s')
        if not math.isfinite(settling_time_s) or settling_time_s < 0.0:
            raise ValueError('settling_time_s must be finite and non-negative')
        if minimum_samples < 2 or minimum_satellites < 1:
            raise ValueError('GPS speed filter sample counts are invalid')
        if enter_confirmations < 1 or exit_confirmations < 1:
            raise ValueError('GPS speed filter confirmations must be positive')
        bounded_values = (
            minimum_path_efficiency,
            minimum_course_coherence,
            minimum_speed_agreement,
            smoothing_alpha,
        )
        if not all(
            math.isfinite(value) and 0.0 < value <= 1.0
            for value in bounded_values
        ):
            raise ValueError('GPS speed filter ratios must be in (0, 1]')

        self.window_s = window_s
        self.settling_time_s = settling_time_s
        self.minimum_span_s = minimum_span_s
        self.minimum_samples = minimum_samples
        self.minimum_displacement_m = minimum_displacement_m
        self.hdop_displacement_scale_m = hdop_displacement_scale_m
        self.minimum_path_efficiency = minimum_path_efficiency
        self.minimum_course_coherence = minimum_course_coherence
        self.minimum_speed_agreement = minimum_speed_agreement
        self.maximum_hdop = maximum_hdop
        self.minimum_satellites = minimum_satellites
        self.require_quality = bool(require_quality)
        self.enter_confirmations = enter_confirmations
        self.exit_confirmations = exit_confirmations
        self.smoothing_alpha = smoothing_alpha
        self.maximum_speed_mps = maximum_speed_mps

        self._samples: Deque[_Sample] = deque()
        self._started_at: Optional[float] = None
        self._moving = False
        self._enter_count = 0
        self._exit_count = 0
        self._filtered_speed = 0.0

    def reset(self) -> None:
        self._samples.clear()
        self._started_at = None
        self._moving = False
        self._enter_count = 0
        self._exit_count = 0
        self._filtered_speed = 0.0

    @staticmethod
    def _optional_float(value: Optional[float]) -> Optional[float]:
        if value is None:
            return None
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None

    def update(
        self,
        *,
        time_s: float,
        latitude: float,
        longitude: float,
        speed_mps: float,
        course_deg: Optional[float] = None,
        hdop: Optional[float] = None,
        satellites: Optional[int] = None,
    ) -> GpsSpeedEstimate:
        values = (time_s, latitude, longitude, speed_mps)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('GPS speed filter input must be finite')
        if speed_mps < 0.0:
            raise ValueError('GPS speed must not be negative')
        if self._samples and time_s <= self._samples[-1].time_s:
            raise ValueError('GPS speed filter time must increase')

        parsed_hdop = self._optional_float(hdop)
        parsed_satellites = (
            int(satellites) if satellites is not None else None
        )
        sample = _Sample(
            time_s=float(time_s),
            latitude=float(latitude),
            longitude=float(longitude),
            speed_mps=min(float(speed_mps), self.maximum_speed_mps),
            course_deg=self._optional_float(course_deg),
            hdop=parsed_hdop,
            satellites=parsed_satellites,
        )
        if self._started_at is None:
            self._started_at = sample.time_s
        self._samples.append(sample)
        oldest_time = time_s - self.window_s
        while len(self._samples) > 1 and self._samples[0].time_s < oldest_time:
            self._samples.popleft()

        first = self._samples[0]
        last = self._samples[-1]
        span_s = max(0.0, last.time_s - first.time_s)
        displacement_m = _distance_m(
            first.latitude,
            first.longitude,
            last.latitude,
            last.longitude,
        )
        samples = list(self._samples)
        path_m = sum(
            _distance_m(
                previous.latitude,
                previous.longitude,
                current.latitude,
                current.longitude,
            )
            for previous, current in zip(
                samples,
                samples[1:],
            )
        )
        path_efficiency = (
            min(1.0, displacement_m / path_m) if path_m > 0.0 else 0.0
        )
        course_coherence = _course_coherence(
            item.course_deg for item in self._samples
        )
        raw_speed_median = statistics.median(
            item.speed_mps for item in self._samples
        )
        position_speed = displacement_m / span_s if span_s > 0.0 else 0.0
        fastest_speed = max(raw_speed_median, position_speed)
        speed_agreement = (
            min(raw_speed_median, position_speed) / fastest_speed
            if fastest_speed > 0.0
            else 1.0
        )
        hdop_values = [
            item.hdop
            for item in self._samples
            if item.hdop is not None and item.hdop > 0.0
        ]
        median_hdop = statistics.median(hdop_values) if hdop_values else None
        movement_threshold_m = max(
            self.minimum_displacement_m,
            (median_hdop or 0.0) * self.hdop_displacement_scale_m,
        )
        quality_available = (
            parsed_hdop is not None and parsed_satellites is not None
        )
        quality_ok = (
            (quality_available or not self.require_quality)
            and (parsed_hdop is None or parsed_hdop <= self.maximum_hdop)
            and (
                parsed_satellites is None
                or parsed_satellites >= self.minimum_satellites
            )
        )
        ready = (
            len(self._samples) >= self.minimum_samples
            and span_s >= self.minimum_span_s
        )
        settling = (
            self._started_at is not None
            and last.time_s - self._started_at < self.settling_time_s
        )
        course_consistent = (
            math.isnan(course_coherence)
            or course_coherence >= self.minimum_course_coherence
        )
        moving_evidence = (
            ready
            and not settling
            and quality_ok
            and displacement_m >= movement_threshold_m
            and path_efficiency >= self.minimum_path_efficiency
            and course_consistent
            and speed_agreement >= self.minimum_speed_agreement
        )

        if settling:
            self._moving = False
            self._enter_count = 0
            self._exit_count = 0
        elif moving_evidence:
            self._enter_count += 1
            self._exit_count = 0
            if self._enter_count >= self.enter_confirmations:
                self._moving = True
        elif ready:
            self._enter_count = 0
            self._exit_count += 1
            if self._exit_count >= self.exit_confirmations:
                self._moving = False

        if self._moving:
            candidate = 0.7 * raw_speed_median + 0.3 * position_speed
            candidate = min(self.maximum_speed_mps, max(0.0, candidate))
            if self._filtered_speed <= 0.0:
                self._filtered_speed = candidate
            else:
                self._filtered_speed += self.smoothing_alpha * (
                    candidate - self._filtered_speed
                )
        else:
            self._filtered_speed = 0.0

        state = (
            'moving'
            if self._moving
            else ('stationary' if ready else 'initializing')
        )
        if settling:
            state = 'settling'
        if ready and not quality_ok:
            state = 'degraded'
        return GpsSpeedEstimate(
            raw_speed_mps=float(speed_mps),
            filtered_speed_mps=self._filtered_speed,
            moving=self._moving,
            state=state,
            sample_count=len(self._samples),
            window_span_s=span_s,
            displacement_m=displacement_m,
            path_efficiency=path_efficiency,
            course_coherence=course_coherence,
            raw_speed_median_mps=raw_speed_median,
            position_speed_mps=position_speed,
            speed_agreement=speed_agreement,
            movement_threshold_m=movement_threshold_m,
            quality_ok=quality_ok,
        )


__all__ = ['GpsSpeedEstimate', 'GpsSpeedFilter']
