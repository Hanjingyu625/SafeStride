"""Small, dependency-free BE-220 NMEA parser used by the Pi ROS node."""
import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class GpsFix:
    latitude: float
    longitude: float
    speed_mps: Optional[float]
    valid: bool


def _coordinate(raw: str, hemisphere: str) -> float:
    if not raw or hemisphere not in {'N', 'S', 'E', 'W'}:
        raise ValueError('invalid NMEA coordinate')
    value = float(raw)
    degrees = int(value // 100)
    minutes = value - degrees * 100
    maximum_degrees = 90 if hemisphere in {'N', 'S'} else 180
    if (
        not math.isfinite(value)
        or degrees > maximum_degrees
        or minutes < 0.0
        or minutes >= 60.0
        or (degrees == maximum_degrees and minutes != 0.0)
    ):
        raise ValueError('NMEA coordinate is out of range')
    result = degrees + minutes / 60.0
    return -result if hemisphere in {'S', 'W'} else result


def parse_fix(sentence: str) -> Optional[GpsFix]:
    """Parse GGA or RMC. Return None for unsupported/corrupt sentences."""
    sentence = sentence.strip()
    if not sentence.startswith('$') or '*' not in sentence:
        return None
    body, supplied = sentence[1:].split('*', 1)
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    try:
        if checksum != int(supplied[:2], 16):
            return None
        fields = body.split(',')
        kind = fields[0][-3:]
        if kind == 'RMC' and len(fields) >= 8:
            valid = fields[2] == 'A'
            if not valid:
                return GpsFix(math.nan, math.nan, None, False)
            return GpsFix(_coordinate(fields[3], fields[4]),
                          _coordinate(fields[5], fields[6]),
                          float(fields[7] or 0.0) * 0.514444, valid)
        if kind == 'GGA' and len(fields) >= 7:
            valid = int(fields[6] or 0) > 0
            if not valid:
                return GpsFix(math.nan, math.nan, None, False)
            return GpsFix(_coordinate(fields[2], fields[3]),
                          _coordinate(fields[4], fields[5]), None, valid)
    except (ValueError, IndexError):
        return None
    return None
