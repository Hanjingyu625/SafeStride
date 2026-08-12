"""Conservative rolling walking-speed profile from the v6 prototype."""

import json
import math
from collections import deque
from pathlib import Path
from typing import Deque, Iterable, Optional


def percentile(values: Iterable[float], fraction: float) -> Optional[float]:
    ordered = sorted(float(value) for value in values if math.isfinite(value))
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    fraction = max(0.0, min(1.0, fraction))
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


class UserSpeedProfile:
    def __init__(
        self,
        path: str = '',
        *,
        default_speed_mps: float = 0.50,
    ) -> None:
        self.path = Path(path).expanduser() if path else None
        self.default_speed_mps = default_speed_mps
        self.samples: Deque[float] = deque(maxlen=300)
        self.load()

    def load(self) -> None:
        if self.path is None or not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text(encoding='utf-8'))
            for value in data.get('recent_speed_samples_mps', []):
                speed = float(value)
                if math.isfinite(speed) and 0.12 <= speed <= 1.8:
                    self.samples.append(speed)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return

    def add(self, speed_mps: Optional[float], *, allow_update: bool = True) -> None:
        if speed_mps is None or not allow_update:
            return
        speed = float(speed_mps)
        if math.isfinite(speed) and 0.12 <= speed <= 1.8:
            self.samples.append(speed)

    def safe_speed(self) -> float:
        value = percentile(self.samples, 0.20)
        if value is None:
            return self.default_speed_mps
        return max(0.30, min(1.00, value))

    def save(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        payload = {
            'sample_count': len(self.samples),
            'safe_speed_mps': round(self.safe_speed(), 3),
            'recent_speed_samples_mps': [
                round(value, 3) for value in list(self.samples)[-120:]
            ],
        }
        temporary.write_text(
            json.dumps(payload, indent=2),
            encoding='utf-8',
        )
        temporary.replace(self.path)


__all__ = ['UserSpeedProfile', 'percentile']
