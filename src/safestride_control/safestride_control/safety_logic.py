"""ROS-independent safety primitives used by the command supervisor."""

import math
from typing import Optional


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


__all__ = ['finite_parameter']
