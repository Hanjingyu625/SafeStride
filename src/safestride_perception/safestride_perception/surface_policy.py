"""Convert advisory YOLO output into a bounded speed scale."""

SCALES = {'smooth': 1.0, 'rough': 0.55, 'wet': 0.40, 'gravel': 0.35,
          'step': 0.0, 'hole': 0.0}


def speed_scale(label: str, confidence: float, threshold: float = 0.65) -> float:
    if not 0.0 <= confidence <= 1.0 or confidence < threshold:
        return 0.0
    return min(1.0, max(0.0, SCALES.get(label.lower(), 0.0)))
