"""Seoul V2X pedestrian-signal response parsing and retrieval."""

import json
import math
from concurrent.futures import ThreadPoolExecutor
from decimal import Decimal, InvalidOperation
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


DEFAULT_TIMING_URL = (
    'https://t-data.seoul.go.kr/apig/apiman-gateway/'
    'tapi/v2xSignalPhaseTimingInformation/1.0'
)
INVALID_SIGNAL_VALUES = {36000, 36001, -1}
DEFAULT_PHASE_URL = (
    'https://t-data.seoul.go.kr/apig/apiman-gateway/'
    'tapi/v2xSignalPhaseInformation/1.0'
)
DEFAULT_COMBINED_URL = (
    'https://t-data.seoul.go.kr/apig/apiman-gateway/'
    'tapi/v2xSignalPhaseTimingFusionCurrentInfo/1.0'
)
OPPOSITE_DIRECTION = {
    'nt': 'st',
    'ne': 'sw',
    'et': 'wt',
    'se': 'nw',
    'st': 'nt',
    'sw': 'ne',
    'wt': 'et',
    'nw': 'se',
}


def find_value(data: Any, key: str) -> Any:
    if isinstance(data, dict):
        if key in data:
            return data[key]
        for value in data.values():
            found = find_value(value, key)
            if found is not None:
                return found
    elif isinstance(data, list):
        for value in data:
            found = find_value(value, key)
            if found is not None:
                return found
    return None


def collect_signal_records(data: Any) -> List[Mapping[str, Any]]:
    records: List[Mapping[str, Any]] = []
    if isinstance(data, dict):
        if (
            data.get('itstId') not in (None, '')
            and data.get('trsmUtcTime') not in (None, '')
        ):
            records.append(data)
        for value in data.values():
            records.extend(collect_signal_records(value))
    elif isinstance(data, list):
        for value in data:
            records.extend(collect_signal_records(value))
    return records


def latest_signal_record(data: Any, intersection_id: str) -> Mapping[str, Any]:
    records = [
        record
        for record in collect_signal_records(data)
        if str(record.get('itstId')) == str(intersection_id)
    ]
    if not records:
        raise ValueError('no signal record matched intersection_id')

    def timestamp(record: Mapping[str, Any]) -> float:
        try:
            return float(record.get('trsmUtcTime', -1.0))
        except (TypeError, ValueError):
            return -1.0

    return max(records, key=timestamp)


def _valid_signal(raw: Any) -> Optional[float]:
    if raw in (None, ''):
        return None
    try:
        deciseconds = int(float(raw))
    except (TypeError, ValueError, OverflowError):
        return None
    if deciseconds in INVALID_SIGNAL_VALUES or deciseconds < 0:
        return None
    return deciseconds / 10.0


def newer_signal_record(candidate: Mapping[str, Any], previous: Optional[Mapping[str, Any]]) -> bool:
    """Do not refresh a cache with duplicate or out-of-order source times."""
    try:
        timestamp = Decimal(str(candidate.get('trsmUtcTime', '')))
        if not timestamp.is_finite():
            return False
        if previous is None:
            return True
        old = Decimal(str(previous.get('trsmUtcTime', '')))
        return old.is_finite() and timestamp > old
    except InvalidOperation:
        return False


def countdown_remaining(remaining_s: float, age_s: float) -> float:
    if not all(math.isfinite(v) and v >= 0.0 for v in (remaining_s, age_s)):
        raise ValueError('invalid signal countdown or age')
    return max(0.0, remaining_s - age_s)


def evaluate_pedestrian_signal(timing, phase, direction, now, max_age_s=12.0):
    """Pair the same signal head, using source epoch milliseconds for freshness."""
    def age(record):
        try:
            value = now - float(record['trsmUtcTime']) / 1000.0
        except (TypeError, KeyError, ValueError, OverflowError):
            raise ValueError('signal source time unavailable')
        if not math.isfinite(value) or value < -1.0 or value > max_age_s:
            raise ValueError('signal source time stale or clock mismatch')
        return max(0.0, value)

    try:
        if direction not in OPPOSITE_DIRECTION:
            raise ValueError('unsupported signal direction')
        age(phase)
        state = phase.get(direction + 'PdsgStatNm')
        if state == 'stop-And-Remain':
            return 0.0, True, 'red pedestrian signal'
        if state not in (
            'protected-Movement-Allowed',
            'permissive-Movement-Allowed',
        ):
            raise ValueError('pedestrian phase unavailable or unsupported')
        timing_age = age(timing)
        if str(timing.get('itstId')) != str(phase.get('itstId')):
            raise ValueError('signal intersection mismatch')
        if abs(float(timing['trsmUtcTime']) - float(phase['trsmUtcTime'])) > 3000:
            raise ValueError('signal phase/countdown timestamps do not match')
        remaining = _valid_signal(timing.get(direction + 'PdsgRmdrCs'))
        if remaining is None:
            raise ValueError('pedestrian countdown unavailable')
        return countdown_remaining(remaining, timing_age), True, 'green pedestrian signal'
    except ValueError as error:
        return None, False, str(error)


def request_signal_bundle(api_key, intersection_id, *, url, phase_url,
                          timeout_s, combined_url=None, direction=None):
    if combined_url:
        try:
            combined = request_signal_data(
                api_key, intersection_id, url=combined_url,
                timeout_s=timeout_s, key_param='apikey')
            pedestrian_states = [
                (key[:-6], value)
                for key, value in combined.items()
                if key.endswith('PdsgStatNm') and value is not None
                and (direction is None or key == direction + 'PdsgStatNm')
            ]
            if pedestrian_states and all(
                state == 'stop-And-Remain'
                or (state in ('protected-Movement-Allowed', 'permissive-Movement-Allowed')
                    and _valid_signal(combined.get(prefix + 'RmdrCs')) is not None)
                for prefix, state in pedestrian_states
            ):
                return {'timing': combined, 'phase': combined}
        except (RuntimeError, ValueError):
            pass
    # Independent results allow a confirmed red even when countdown retrieval fails.
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {name: pool.submit(request_signal_data, api_key, intersection_id,
                   url=endpoint, timeout_s=timeout_s)
                   for name, endpoint in (('timing', url), ('phase', phase_url))}
        result = {}
        for name, future in futures.items():
            try:
                result[name] = future.result()
            except Exception as error:
                result[name] = None
                result[name + '_error'] = str(error)
        return result


def signal_remaining_for_crosswalk(
    data: Any,
    direction: str,
) -> Tuple[Tuple[float, str], Dict[str, Any]]:
    """Return the shorter valid value for the two opposing signal fields."""

    if direction not in OPPOSITE_DIRECTION:
        raise ValueError('unsupported signal direction: ' + direction)
    values: List[Tuple[float, str]] = []
    raw_values: Dict[str, Any] = {}
    for candidate in (direction, OPPOSITE_DIRECTION[direction]):
        field = candidate + 'PdsgRmdrCs'
        raw = find_value(data, field)
        raw_values[field] = raw
        parsed = _valid_signal(raw)
        if parsed is not None:
            values.append((parsed, field))
    if not values:
        raise ValueError('no valid pedestrian signal value: ' + str(raw_values))
    return min(values, key=lambda item: item[0]), raw_values


def all_valid_signal_values(data: Any) -> Iterable[Tuple[float, str]]:
    for direction in OPPOSITE_DIRECTION:
        field = direction + 'PdsgRmdrCs'
        parsed = _valid_signal(find_value(data, field))
        if parsed is not None:
            yield parsed, field


def request_signal_data(
    api_key: str,
    intersection_id: str,
    *,
    url: str = DEFAULT_TIMING_URL,
    timeout_s: float = 10.0,
    key_param: str = 'apiKey',
) -> Mapping[str, Any]:
    """Fetch and select the latest record for one intersection."""

    if not api_key:
        raise ValueError('API key is empty')
    if not intersection_id:
        raise ValueError('intersection_id is empty')
    query = urllib.parse.urlencode(
        {
            key_param: api_key,
            'itstId': intersection_id,
            'type': 'json',
            'pageNo': 1,
            'numOfRows': 100,
        }
    )
    request = urllib.request.Request(
        url + '?' + query,
        headers={'User-Agent': 'safestride-crosswalk/1.0'},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = response.read().decode('utf-8-sig')
    except urllib.error.HTTPError as error:
        detail = error.read().decode('utf-8', errors='replace')
        raise RuntimeError('signal API HTTP %s: %s' % (error.code, detail[:160]))
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError('signal API request failed: %s' % error) from error
    try:
        parsed = json.loads(body)
    except json.JSONDecodeError as error:
        raise ValueError('signal API response is not JSON') from error
    return latest_signal_record(parsed, intersection_id)


__all__ = [
    'DEFAULT_COMBINED_URL',
    'DEFAULT_TIMING_URL',
    'all_valid_signal_values',
    'collect_signal_records',
    'latest_signal_record',
    'request_signal_data',
    'signal_remaining_for_crosswalk',
]
