"""Convert road-surface classifier output into a bounded speed scale."""

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


def speed_scale(
    label: str,
    confidence: float,
    threshold: float = 0.65,
) -> float:
    if not 0.0 <= confidence <= 1.0 or confidence < threshold:
        return 0.0
    return min(1.25, max(0.0, SCALES.get(label.lower(), 0.0)))
