"""Fail-fast helpers for safety-critical ROS parameter values."""

import math
from typing import Optional


def finite_float(
    name: str,
    value,
    *,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
    minimum_inclusive: bool = True,
) -> float:
    """Return a finite float within the requested bounds."""

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


def bounded_int(
    name: str,
    value,
    *,
    minimum: int,
    maximum: int,
) -> int:
    """Return an exact, non-boolean integer within inclusive bounds."""

    if isinstance(value, bool):
        raise ValueError(f'{name} must be an integer, not bool')
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{name} must be an integer') from error
    if parsed != value:
        raise ValueError(f'{name} must be an exact integer')
    if not minimum <= parsed <= maximum:
        raise ValueError(
            f'{name} must be between {minimum} and {maximum}'
        )
    return parsed
