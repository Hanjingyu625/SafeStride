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
    course_deg: Optional[float] = None
    sentence_type: str = ''
    fix_quality: Optional[int] = None
    satellites: Optional[int] = None
    hdop: Optional[float] = None
    altitude_m: Optional[float] = None


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
        if kind == 'RMC' and len(fields) >= 9:
            valid = fields[2] == 'A'
            if not valid:
                return GpsFix(
                    math.nan,
                    math.nan,
                    None,
                    False,
                    sentence_type='RMC',
                )
            speed_mps = (
                float(fields[7]) * 0.514444 if fields[7] else None
            )
            if speed_mps is not None and (
                not math.isfinite(speed_mps) or speed_mps < 0.0
            ):
                raise ValueError('invalid NMEA speed')
            course_deg = float(fields[8]) % 360.0 if fields[8] else None
            if course_deg is not None and not math.isfinite(course_deg):
                raise ValueError('invalid NMEA course')
            return GpsFix(
                _coordinate(fields[3], fields[4]),
                _coordinate(fields[5], fields[6]),
                speed_mps,
                valid,
                course_deg,
                sentence_type='RMC',
            )
        if kind == 'GGA' and len(fields) >= 7:
            fix_quality = int(fields[6] or 0)
            satellites = (
                int(fields[7]) if len(fields) > 7 and fields[7] else None
            )
            hdop = float(fields[8]) if len(fields) > 8 and fields[8] else None
            altitude_m = (
                float(fields[9]) if len(fields) > 9 and fields[9] else None
            )
            if satellites is not None and not 0 <= satellites <= 200:
                raise ValueError('invalid NMEA satellite count')
            if hdop is not None and (
                not math.isfinite(hdop) or hdop < 0.0
            ):
                raise ValueError('invalid NMEA HDOP')
            if altitude_m is not None and not math.isfinite(altitude_m):
                raise ValueError('invalid NMEA altitude')
            valid = fix_quality > 0
            if not valid:
                return GpsFix(
                    math.nan,
                    math.nan,
                    None,
                    False,
                    sentence_type='GGA',
                    fix_quality=fix_quality,
                    satellites=satellites,
                    hdop=hdop,
                    altitude_m=altitude_m,
                )
            return GpsFix(
                _coordinate(fields[2], fields[3]),
                _coordinate(fields[4], fields[5]),
                None,
                valid,
                sentence_type='GGA',
                fix_quality=fix_quality,
                satellites=satellites,
                hdop=hdop,
                altitude_m=altitude_m,
            )
    except (ValueError, IndexError):
        return None
    return None
