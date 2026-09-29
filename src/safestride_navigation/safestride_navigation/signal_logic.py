"""Seoul V2X pedestrian-signal response parsing and retrieval."""

import json
import math
import os
import time
import threading
from pathlib import Path
from email.utils import parsedate_to_datetime
from concurrent.futures import ThreadPoolExecutor
from decimal import Decimal, InvalidOperation
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


DEFAULT_TIMING_URL = (
    'https://t-data.seoul.go.kr/apig/apiman-gateway/'
    'tapi/v2xSignalPhaseTimingCurrentInfo/1.0'
)
INVALID_SIGNAL_VALUES = {36000, 36001, -1}
DEFAULT_PHASE_URL = (
    'https://t-data.seoul.go.kr/apig/apiman-gateway/'
    'tapi/v2xSignalPhaseCurrentInfo/1.0'
)
DEFAULT_COMBINED_URL = ''  # The two separately approved current APIs are the default.
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


def current_signal_url(url):
    """Migrate only the two known historical Seoul signal endpoints."""
    base = 'https://t-data.seoul.go.kr/apig/apiman-gateway/tapi/'
    return {
        base + 'v2xSignalPhaseTimingInformation/1.0': DEFAULT_TIMING_URL,
        base + 'v2xSignalPhaseInformation/1.0': DEFAULT_PHASE_URL,
    }.get(url, url)


def load_signal_api_key(path=''):
    """Use a saved key on every startup; explicit configuration has priority."""
    configured = path or os.environ.get('SAFESTRIDE_SIGNAL_API_KEY_FILE', '')
    candidates = [Path(configured).expanduser()] if configured else [
        Path('/etc/safestride/signal_api_key.txt'),
        Path(__file__).resolve().parents[3] / 'raspberry_pi/api_key.txt',
    ]
    for source in candidates:
        try:
            key = source.read_text(encoding='utf-8-sig').strip()
        except (OSError, UnicodeError):
            continue
        if key and 'CHANGE_ME' not in key:
            return key
    return ''


def retry_after_seconds(value, now=None):
    try:
        seconds = float(value)
    except (TypeError, ValueError):
        try:
            seconds = parsedate_to_datetime(value).timestamp() - (time.time() if now is None else now)
        except (TypeError, ValueError, OverflowError):
            return 300.0
    return max(1.0, seconds) if math.isfinite(seconds) else 300.0


def rate_limit_info(headers, body='', *, now=None):
    """Seoul's gateway puts its quota headers inside the JSON error body."""
    values = {}
    try:
        nested = json.loads(body).get('headers', {}) if body else {}
        if isinstance(nested, dict):
            values.update({str(k).lower(): v for k, v in nested.items()})
    except (ValueError, AttributeError):
        pass
    for name in ('X-RateLimit-Limit', 'X-RateLimit-Remaining', 'X-RateLimit-Reset', 'Retry-After'):
        value = headers.get(name) if headers is not None else None
        if isinstance(value, (str, int, float)):
            values[name.lower()] = value
    result = {}
    for name, field in (('limit', 'x-ratelimit-limit'),
                        ('remaining', 'x-ratelimit-remaining'),
                        ('reset_s', 'x-ratelimit-reset')):
        try:
            value = float(values[field])
            if math.isfinite(value):
                result[name] = value
        except (KeyError, ValueError, TypeError):
            pass
    if 'reset_s' in result:
        current = time.time() if now is None else now
        if result['reset_s'] > 1_000_000_000:
            result['reset_s'] -= current
        result['reset_s'] = max(1.0, result['reset_s'])
    if 'retry-after' in values:
        result['retry_s'] = retry_after_seconds(values['retry-after'], now=now)
    return result


class SignalApiError(RuntimeError):
    def __init__(self, message, *, status=None, retry_after_s=3.0):
        super().__init__(message)
        self.status = status
        self.retry_after_s = retry_after_s


class SignalApiClient:
    """Endpoint-wide backoff, including when the selected intersection changes."""
    def __init__(self, *, clock=time.monotonic, fetch=None, minimum_interval_s=1.0,
                 cache_file=None):
        if not math.isfinite(minimum_interval_s) or minimum_interval_s <= 0:
            raise ValueError('minimum_interval_s must be finite and positive')
        self._minimum_interval_s = minimum_interval_s
        self._clock = clock
        self._fetch = fetch or request_signal_data
        self._blocked = {}
        self._lock = threading.Lock()
        self._cache_file = Path(cache_file).expanduser() if cache_file else None
        if self._cache_file:
            try:
                saved = json.loads(self._cache_file.read_text(encoding='utf-8'))
                for url, item in saved.items():
                    remaining = float(item['until']) - time.time()
                    if math.isfinite(remaining) and remaining > 0:
                        self._blocked[url] = (self._clock() + remaining, str(item['reason']))
            except (OSError, ValueError, TypeError, KeyError, AttributeError):
                pass

    def _save_backoff(self):
        if self._cache_file is None:
            return
        saved = {url: dict(until=time.time() + until - self._clock(), reason=reason)
                 for url, (until, reason) in self._blocked.items()
                 if until > self._clock() and reason != 'signal poll interval'}
        try:
            self._cache_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._cache_file.with_suffix('.tmp')
            temporary.write_text(json.dumps(saved), encoding='utf-8')
            temporary.replace(self._cache_file)
        except OSError:
            pass

    def retry_remaining(self, urls):
        with self._lock:
            return max([0.0] + [self._blocked.get(current_signal_url(url), (0.0, ''))[0]
                                - self._clock() for url in urls if url])

    def poll_interval_remaining(self, urls):
        """Poll when at least one endpoint is due; a blocked timer must not spin."""
        with self._lock:
            remaining = [max(0.0, self._blocked.get(current_signal_url(url), (0.0, ''))[0]
                             - self._clock()) for url in urls if url]
        return min(remaining, default=0.0)

    def wait_reason(self, urls):
        with self._lock:
            reasons = [('%s; retry in %.0fs' % (reason, until - self._clock()))
                       for url in urls if url
                       for until, reason in [self._blocked.get(current_signal_url(url), (0.0, ''))]
                       if until > self._clock() and reason != 'signal poll interval']
        return '; '.join(dict.fromkeys(reasons))

    def get(self, api_key, intersection_id, *, url, timeout_s, **kwargs):
        url = current_signal_url(url)
        now = self._clock()
        with self._lock:
            until, reason = self._blocked.get(url, (0.0, ''))
            if now < until:
                raise SignalApiError('%s; retry in %.1fs' % (reason, until - now),
                                     retry_after_s=until - now)
            self._blocked[url] = (now + self._minimum_interval_s, 'signal poll interval')
        try:
            record = self._fetch(api_key, intersection_id, url=url, timeout_s=timeout_s, **kwargs)
            quota = record.get('_api_rate_limit', {})
            if quota.get('reset_s', 0) > 0 and 'remaining' in quota:
                interval = max(self._minimum_interval_s,
                               quota['reset_s'] / max(quota['remaining'], 1.0))
                if interval > self._minimum_interval_s:
                    with self._lock:
                        self._blocked[url] = (self._clock() + interval, 'signal API quota pacing')
                        self._save_backoff()
            return record
        except SignalApiError as error:
            with self._lock:
                self._blocked[url] = (self._clock() + error.retry_after_s, str(error))
                self._save_backoff()
            raise


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


def evaluate_crosswalk_signal(timing, phase, directions, now, max_age_s=12.0):
    """An unresolved arm requires agreement of every geometrically plausible head."""
    if not directions:
        return None, False, 'crosswalk signal mapping unavailable'
    results = [evaluate_pedestrian_signal(timing, phase, direction, now, max_age_s)
               for direction in directions]
    for direction, (_, valid, reason) in zip(directions, results):
        if not valid:
            return None, False, 'signal head %s: %s' % (direction, reason)
    if len({reason for _, _, reason in results}) != 1:
        return None, False, 'ambiguous signal heads disagree; cannot determine this crosswalk signal'
    return min(value for value, _, _ in results), True, results[0][2]


def signal_lookup_active(crosswalk, maximum_distance_m):
    """Only spend live-signal quota close to the selected crosswalk."""
    if not crosswalk or not math.isfinite(maximum_distance_m) or maximum_distance_m <= 0:
        return False
    try:
        distance = float(crosswalk.get('edge_distance_m'))
    except (TypeError, ValueError):
        return False
    return math.isfinite(distance) and distance >= 0 and distance <= maximum_distance_m


def timing_required_for_phase(phase, directions):
    """Red needs only phase data; a possible green also needs countdown data."""
    if not phase:
        return False
    green = {'protected-Movement-Allowed', 'permissive-Movement-Allowed'}
    return any(phase.get(direction + 'PdsgStatNm') in green
               for direction in directions)


def request_signal_bundle(api_key, intersection_id, *, url, phase_url,
                          timeout_s, combined_url=None, direction=None, client=None,
                          timing_required=True):
    fetch = client.get if client is not None else request_signal_data
    combined_error = ''
    if combined_url:
        try:
            combined = fetch(
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
        except (RuntimeError, ValueError) as error:
            combined_error = str(error)
    # Independent results allow a confirmed red even when countdown retrieval fails.
    endpoints = [('phase', phase_url)]
    if timing_required:
        endpoints.insert(0, ('timing', url))
    with ThreadPoolExecutor(max_workers=len(endpoints)) as pool:
        futures = {name: pool.submit(fetch, api_key, intersection_id,
                   url=endpoint, timeout_s=timeout_s)
                   for name, endpoint in endpoints}
        result = {}
        if combined_error:
            result['combined_error'] = combined_error
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
    key_param: Optional[str] = None,
) -> Mapping[str, Any]:
    """Fetch and select the latest record for one intersection."""

    if not api_key:
        raise ValueError('API key is empty')
    if not intersection_id:
        raise ValueError('intersection_id is empty')
    url = current_signal_url(url)
    if key_param is None:
        key_param = 'apikey' if 'CurrentInfo/' in url else 'apiKey'
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
            quota = rate_limit_info(response.headers)
    except urllib.error.HTTPError as error:
        detail = error.read().decode('utf-8', errors='replace')
        detail = detail.replace(api_key, '[REDACTED]').replace(
            urllib.parse.quote_plus(api_key), '[REDACTED]')
        quota = rate_limit_info(error.headers, detail)
        delay = (max(quota.get('retry_s', 0), quota.get('reset_s', 0)) or 300.0
                 if error.code == 429 else
                 300.0 if error.code in (401, 403, 404) else 5.0)
        raise SignalApiError('signal API HTTP %s: %s' % (error.code, detail[:240]),
                             status=error.code, retry_after_s=delay) from None
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise SignalApiError('signal API transport failure: ' + type(error).__name__,
                             retry_after_s=5.0) from None
    try:
        parsed = json.loads(body)
    except json.JSONDecodeError as error:
        raise ValueError('signal API response is not JSON') from error
    record = dict(latest_signal_record(parsed, intersection_id))
    if quota:
        record['_api_rate_limit'] = quota
    return record


__all__ = [
    'DEFAULT_COMBINED_URL',
    'DEFAULT_TIMING_URL',
    'all_valid_signal_values',
    'collect_signal_records',
    'latest_signal_record',
    'request_signal_data',
    'signal_remaining_for_crosswalk',
]
