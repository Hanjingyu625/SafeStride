"""Seoul V2X pedestrian-signal response parsing and retrieval."""

import json
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


DEFAULT_TIMING_URL = (
    'https://t-data.seoul.go.kr/apig/apiman-gateway/'
    'tapi/v2xSignalPhaseTimingInformation/1.0'
)
INVALID_SIGNAL_VALUES = {36000, 36001, -1}
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
    except (TypeError, ValueError):
        return None
    if deciseconds in INVALID_SIGNAL_VALUES or deciseconds < 0:
        return None
    return deciseconds / 10.0


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
) -> Mapping[str, Any]:
    """Fetch and select the latest record for one intersection."""

    if not api_key:
        raise ValueError('API key is empty')
    if not intersection_id:
        raise ValueError('intersection_id is empty')
    query = urllib.parse.urlencode(
        {
            'apiKey': api_key,
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
    'DEFAULT_TIMING_URL',
    'all_valid_signal_values',
    'collect_signal_records',
    'latest_signal_record',
    'request_signal_data',
    'signal_remaining_for_crosswalk',
]
