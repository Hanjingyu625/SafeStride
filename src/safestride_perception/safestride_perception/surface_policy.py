"""Validate road-surface predictions and convert them to speed limits."""

import math

SCALES = {
    # The classifier retains both labels for diagnostics, but their dominant
    # mutual confusion must not make the commanded speed oscillate.
    'smooth': 1.00,
    'smooth_paved': 1.00,
    'rough': 1.00,
    'rough_paved': 1.00,
    'block_paved': 0.95,
    'wet': 0.85,
    'wet_paved': 0.85,
    'wet_unpaved': 0.85,
    'gravel': 0.95,
    'mud_dirt': 1.00,
    'unpaved_mixed': 1.00,
    'snow_ice': 0.75,
    'step': 0.0,
    'hole': 0.0,
}


def prediction_is_confident(
    confidence: float,
    runner_up_confidence: float,
    threshold: float = 0.65,
    min_margin: float = 0.15,
) -> bool:
    """Accept a prediction only when top-1 is strong and unambiguous."""
    values = (confidence, runner_up_confidence, threshold, min_margin)
    if not all(math.isfinite(value) for value in values):
        return False
    if not all(0.0 <= value <= 1.0 for value in values):
        return False
    return (
        confidence >= threshold
        and confidence - runner_up_confidence >= min_margin
    )


def speed_scale(
    label: str,
    confidence: float,
    threshold: float = 0.65,
) -> float:
    if not 0.0 <= confidence <= 1.0 or confidence < threshold:
        return 0.0
    return min(1.25, max(0.0, SCALES.get(label.lower(), 0.0)))
