"""ROS-independent safety primitives used by the command supervisor."""

from dataclasses import dataclass
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


def apply_command_deadband(
    linear: float,
    angular: float,
    linear_threshold: float,
    angular_threshold: float,
) -> Tuple[float, float]:
    """Return command axes with values inside their neutral zone set to zero."""

    if abs(linear) <= linear_threshold:
        linear = 0.0
    if abs(angular) <= angular_threshold:
        angular = 0.0
    return linear, angular


@dataclass
class PostArmNeutralGate:
    """Require a newly received neutral input before permitting motion.

    ``generation`` is incremented by the caller for every raw intent callback.
    A synthesized zero or a command cached before the interlock was reset can
    therefore never qualify as the required operator-neutral observation.
    """

    linear_threshold: float
    angular_threshold: float
    required_after_generation: int = 0
    neutral_seen: bool = False

    def require_new_neutral(self, current_generation: int) -> None:
        """Close the gate until a newer neutral sample is observed."""

        self.required_after_generation = int(current_generation)
        self.neutral_seen = False

    def observe(
        self,
        generation: int,
        linear: float,
        angular: float,
    ) -> bool:
        """Return whether motion may pass on this evaluation cycle.

        The qualifying neutral sample itself is held at zero for one cycle.
        This keeps the transition deterministic even at a deadband boundary.
        """

        if self.neutral_seen:
            return True
        if int(generation) <= self.required_after_generation:
            return False
        if not math.isfinite(linear) or not math.isfinite(angular):
            return False
        if (
            abs(linear) > self.linear_threshold
            or abs(angular) > self.angular_threshold
        ):
            return False
        self.neutral_seen = True
        return False
