"""Validate road-surface predictions and convert them to speed limits."""

import math

SCALES = {
    'smooth': 1.20,
    'smooth_paved': 1.20,
    'rough': 0.70,
    'rough_paved': 0.70,
    'block_paved': 0.65,
    'wet': 0.50,
    'wet_paved': 0.50,
    'wet_unpaved': 0.40,
    'gravel': 0.55,
    'mud_dirt': 0.40,
    'unpaved_mixed': 0.50,
    'snow_ice': 0.0,
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
