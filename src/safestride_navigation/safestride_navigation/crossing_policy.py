"""ROS-independent automatic-crossing state machine adapted from v6."""

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Deque, Dict, Mapping, Optional, Tuple

from .crosswalk_data import evaluate_locked_crosswalk


@dataclass(frozen=True)
class CrossingParameters:
    approach_distance_m: float = 50.0
    lock_distance_m: float = 18.0
    curb_zone_m: float = 7.0
    curb_release_distance_m: float = 12.0
    entry_start_progress_m: float = 1.2
    entry_start_min_gain_m: float = 1.0
    entry_start_window_s: float = 4.0
    entry_min_speed_mps: float = 0.15
    exit_clearance_m: float = 1.5
    exit_hold_s: float = 2.0
    complete_hold_s: float = 4.0
    crossing_timeout_s: float = 180.0
    reaction_time_s: float = 2.0
    entry_safety_margin_s: float = 5.0
    crossing_time_margin_s: float = 2.0
    minimum_estimate_speed_mps: float = 0.15
    maximum_assist_speed_mps: float = 0.85


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class CrossingStateMachine:
    STATES = (
        'IDLE',
        'APPROACHING',
        'WAIT_AT_CURB',
        'ENTRY_ALLOWED',
        'CROSSING',
        'CROSSING_URGENT',
        'EXITING',
    )

    def __init__(
        self,
        parameters: CrossingParameters = CrossingParameters(),
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.parameters = parameters
        self._clock = clock
        self.state = 'IDLE'
        self.state_since = self._clock()
        self.locked_crosswalk: Optional[Dict[str, Any]] = None
        self.locked_intersection_id = ''
        self.progress_history: Deque[Tuple[float, float]] = deque(maxlen=12)
        self.exit_seen_since: Optional[float] = None
        self.crossing_started_at: Optional[float] = None
        self.arm_wheel_origin: Optional[float] = None
        self.crossing_wheel_origin: Optional[float] = None
        self.reason = 'waiting for a crosswalk'

    def set_state(self, new_state: str, reason: str) -> None:
        if new_state not in self.STATES:
            raise ValueError('invalid crossing state: ' + str(new_state))
        if new_state != self.state:
            self.state = new_state
            self.state_since = self._clock()
            if new_state in ('WAIT_AT_CURB', 'ENTRY_ALLOWED'):
                self.arm_wheel_origin = None
            if new_state in ('CROSSING', 'CROSSING_URGENT'):
                if self.crossing_started_at is None:
                    self.crossing_started_at = self.state_since
            if new_state == 'EXITING':
                self.exit_seen_since = self.state_since
        self.reason = reason

    def lock(self, crosswalk: Mapping[str, Any], intersection_id: str) -> None:
        if self.locked_crosswalk is None:
            self.locked_crosswalk = dict(crosswalk)
            self.locked_intersection_id = str(intersection_id or '')
            self.progress_history.clear()

    def reset(self, reason: str = 'reset') -> None:
        self.state = 'IDLE'
        self.state_since = self._clock()
        self.locked_crosswalk = None
        self.locked_intersection_id = ''
        self.progress_history.clear()
        self.exit_seen_since = None
        self.crossing_started_at = None
        self.arm_wheel_origin = None
        self.crossing_wheel_origin = None
        self.reason = reason

    def current_crosswalk(
        self,
        candidate: Optional[Mapping[str, Any]],
        latitude: float,
        longitude: float,
    ) -> Optional[Dict[str, Any]]:
        if self.locked_crosswalk is not None:
            return evaluate_locked_crosswalk(
                self.locked_crosswalk,
                latitude,
                longitude,
            )
        return None if candidate is None else dict(candidate)

    def _record_progress(self, progress_m: float) -> None:
        now = self._clock()
        self.progress_history.append((now, progress_m))
        while (
            self.progress_history
            and now - self.progress_history[0][0]
            > self.parameters.entry_start_window_s
        ):
            self.progress_history.popleft()

    def _automatic_start_detected(
        self,
        progress_m: float,
        speed_mps: Optional[float],
        wheel_distance_m: Optional[float],
    ) -> bool:
        self._record_progress(progress_m)
        if (
            speed_mps is None
            or not math.isfinite(speed_mps)
            or speed_mps < self.parameters.entry_min_speed_mps
        ):
            return False
        if self.arm_wheel_origin is None and wheel_distance_m is not None:
            self.arm_wheel_origin = wheel_distance_m

        gps_started = False
        if (
            progress_m >= self.parameters.entry_start_progress_m
            and len(self.progress_history) >= 2
        ):
            gain = progress_m - min(value for _, value in self.progress_history)
            gps_started = gain >= self.parameters.entry_start_min_gain_m

        wheel_motion_started = False
        if wheel_distance_m is not None and self.arm_wheel_origin is not None:
            wheel_motion_started = (
                wheel_distance_m - self.arm_wheel_origin
                >= self.parameters.entry_start_min_gain_m
            )
        return gps_started or wheel_motion_started

    def update(
        self,
        *,
        candidate: Optional[Mapping[str, Any]],
        intersection_id: str,
        latitude: float,
        longitude: float,
        signal_remaining_s: Optional[float],
        signal_valid: bool,
        safe_speed_mps: float,
        measured_speed_mps: Optional[float],
        wheel_distance_m: Optional[float] = None,
    ) -> Tuple[Optional[Dict[str, Any]], Optional[float], Optional[float]]:
        """Advance the policy and return active crossing, entry time and ETA."""

        now = self._clock()
        active = self.current_crosswalk(candidate, latitude, longitude)
        if active is None:
            self.reset('no crosswalk candidate')
            return None, None, None

        safe_speed_mps = max(
            float(safe_speed_mps),
            self.parameters.minimum_estimate_speed_mps,
        )
        required_entry_time = (
            float(active['length_m']) / safe_speed_mps
            + self.parameters.reaction_time_s
            + self.parameters.entry_safety_margin_s
        )

        if self.state == 'IDLE':
            if active['edge_distance_m'] <= self.parameters.approach_distance_m:
                self.set_state('APPROACHING', 'crosswalk detected ahead')
            else:
                self.reason = 'crosswalk is outside approach range'

        if self.state == 'APPROACHING':
            if (
                active['edge_distance_m']
                > self.parameters.approach_distance_m + 10.0
            ):
                self.reset('moved away from crosswalk')
                return active, required_entry_time, None
            if (
                self.locked_crosswalk is None
                and active['edge_distance_m'] <= self.parameters.lock_distance_m
            ):
                selected_id = str(
                    active.get('intersection_id') or intersection_id or ''
                )
                self.lock(active, selected_id)
                active = self.current_crosswalk(candidate, latitude, longitude)
                assert active is not None
            if active['edge_distance_m'] <= self.parameters.curb_zone_m:
                if (
                    signal_valid
                    and signal_remaining_s is not None
                    and signal_remaining_s >= required_entry_time
                ):
                    self.set_state(
                        'ENTRY_ALLOWED',
                        'enough signal time to enter',
                    )
                else:
                    self.set_state(
                        'WAIT_AT_CURB',
                        'wait for a safer signal window',
                    )

        elif self.state == 'WAIT_AT_CURB':
            progress = active.get('progress_m')
            if active['edge_distance_m'] > self.parameters.curb_release_distance_m:
                self.set_state('APPROACHING', 'user moved away from curb')
            elif (
                progress is not None
                and self._automatic_start_detected(
                    float(progress),
                    measured_speed_mps,
                    wheel_distance_m,
                )
            ):
                self.crossing_wheel_origin = self.arm_wheel_origin
                self.set_state(
                    'CROSSING_URGENT',
                    'entry detected while waiting; continue across',
                )
            elif (
                signal_valid
                and signal_remaining_s is not None
                and signal_remaining_s >= required_entry_time
            ):
                self.progress_history.clear()
                self.set_state(
                    'ENTRY_ALLOWED',
                    'signal time became sufficient',
                )

        elif self.state == 'ENTRY_ALLOWED':
            progress = active.get('progress_m')
            if active['edge_distance_m'] > self.parameters.curb_release_distance_m:
                self.set_state('APPROACHING', 'user moved away before entry')
            elif (
                not signal_valid
                or signal_remaining_s is None
                or signal_remaining_s < required_entry_time
            ):
                self.set_state(
                    'WAIT_AT_CURB',
                    'signal window closed before entry',
                )
            elif progress is not None and self._automatic_start_detected(
                float(progress),
                measured_speed_mps,
                wheel_distance_m,
            ):
                self.crossing_wheel_origin = self.arm_wheel_origin
                self.set_state(
                    'CROSSING',
                    'automatic entry detected from position and motion',
                )

        elif self.state in ('CROSSING', 'CROSSING_URGENT'):
            progress = active.get('progress_m')
            if (
                wheel_distance_m is not None
                and self.crossing_wheel_origin is not None
            ):
                wheel_progress = max(
                    wheel_distance_m - self.crossing_wheel_origin,
                    0.0,
                )
                progress = (
                    wheel_progress
                    if progress is None
                    else max(float(progress), wheel_progress)
                )
                active['progress_m'] = progress
                active['remaining_m'] = max(
                    float(active['length_m']) - progress,
                    0.0,
                )

            remaining = active.get('remaining_m')
            if progress is not None:
                self._record_progress(float(progress))
            estimate_speed = measured_speed_mps
            if (
                estimate_speed is None
                or not math.isfinite(estimate_speed)
                or estimate_speed < self.parameters.minimum_estimate_speed_mps
            ):
                estimate_speed = max(
                    safe_speed_mps * 0.75,
                    self.parameters.minimum_estimate_speed_mps,
                )
            eta_s = (
                float(remaining) / estimate_speed
                if remaining is not None
                else None
            )

            if (
                progress is not None
                and float(progress)
                >= float(active['length_m']) + self.parameters.exit_clearance_m
            ):
                if self.exit_seen_since is None:
                    self.exit_seen_since = now
                elif now - self.exit_seen_since >= self.parameters.exit_hold_s:
                    self.set_state('EXITING', 'far curb reached')
            else:
                self.exit_seen_since = None

            if self.state in ('CROSSING', 'CROSSING_URGENT'):
                urgent = (
                    not signal_valid
                    or signal_remaining_s is None
                    or eta_s is None
                    or signal_remaining_s
                    < eta_s + self.parameters.crossing_time_margin_s
                )
                self.set_state(
                    'CROSSING_URGENT' if urgent else 'CROSSING',
                    (
                        'continue crossing; remaining signal is tight'
                        if urgent
                        else 'crossing progress is within the signal window'
                    ),
                )
                if (
                    self.crossing_started_at is not None
                    and now - self.crossing_started_at
                    > self.parameters.crossing_timeout_s
                ):
                    self.set_state(
                        'CROSSING_URGENT',
                        'crossing timeout; continue assistance and alert',
                    )
            return active, required_entry_time, eta_s

        elif self.state == 'EXITING':
            if now - self.state_since >= self.parameters.complete_hold_s:
                self.reset('crossing completed')

        return active, required_entry_time, None

    def command(
        self,
        safe_speed_mps: float,
        measured_speed_mps: Optional[float],
    ) -> Dict[str, Any]:
        """Return the high-level command; hardware safety remains downstream."""

        safe_speed = max(float(safe_speed_mps), 0.0)
        measured_speed = (
            measured_speed_mps
            if measured_speed_mps is not None
            and math.isfinite(measured_speed_mps)
            else 0.0
        )
        if self.state == 'IDLE':
            mode, speed, allowed, alert = 'NORMAL_ASSIST', safe_speed, False, 0
        elif self.state == 'APPROACHING':
            mode, speed, allowed, alert = (
                'SPEED_LIMIT', min(safe_speed, 0.50), False, 1
            )
        elif self.state == 'WAIT_AT_CURB':
            mode, speed, allowed, alert = 'SOFT_STOP', 0.0, False, 2
        elif self.state == 'ENTRY_ALLOWED':
            mode, speed, allowed, alert = 'ENTRY_ALLOWED', safe_speed, True, 3
        elif self.state == 'CROSSING':
            mode, speed, allowed, alert = (
                'CROSSING_ASSIST', max(safe_speed, measured_speed), True, 4
            )
        elif self.state == 'CROSSING_URGENT':
            mode, speed, allowed, alert = (
                'CROSSING_URGENT',
                min(
                    max(safe_speed + 0.10, measured_speed),
                    self.parameters.maximum_assist_speed_mps,
                ),
                True,
                5,
            )
        else:
            mode, speed, allowed, alert = (
                'EXITING', min(safe_speed, 0.50), True, 6
            )
        return {
            'mode': mode,
            'target_speed_mps': clamp(
                speed,
                0.0,
                self.parameters.maximum_assist_speed_mps,
            ),
            'entry_allowed': allowed,
            'alert_code': alert,
        }


__all__ = ['CrossingParameters', 'CrossingStateMachine', 'clamp']
