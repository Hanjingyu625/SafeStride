"""Convert road-surface classifier output into a bounded speed scale."""

SCALES = {
    'smooth': 1.0,
    'smooth_paved': 1.0,
    'rough': 0.55,
    'rough_paved': 0.55,
    'block_paved': 0.55,
    'wet': 0.40,
    'wet_paved': 0.40,
    'wet_unpaved': 0.30,
    'gravel': 0.35,
    'mud_dirt': 0.30,
    'unpaved_mixed': 0.35,
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
    return min(1.0, max(0.0, SCALES.get(label.lower(), 0.0)))
