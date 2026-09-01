"""Seoul V2X intersection-map retrieval and nearest-ID selection."""

import json
import math
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional

from .crosswalk_data import haversine_m, number


DEFAULT_INTERSECTION_MAP_URL = (
    'https://t-data.seoul.go.kr/apig/apiman-gateway/'
    'tapi/v2xCrossroadMapInformation/1.0'
)
Intersection = Dict[str, Any]


def _records(document: Any) -> Iterable[Mapping[str, Any]]:
    if isinstance(document, dict):
        if (
            document.get('itstId') not in (None, '')
            or document.get('intersection_id') not in (None, '')
        ):
            yield document
        for value in document.values():
            yield from _records(value)
    elif isinstance(document, list):
        for value in document:
            yield from _records(value)


def normalize_intersections(document: Any) -> List[Intersection]:
    result: Dict[str, Intersection] = {}
    for record in _records(document):
        identifier = str(
            record.get('itstId', record.get('intersection_id', ''))
        ).strip()
        latitude = number(
            record.get('mapCtptIntLat', record.get('latitude'))
        )
        longitude = number(
            record.get('mapCtptIntLot', record.get('longitude'))
        )
        if (
            not identifier
            or latitude is None
            or longitude is None
            or not -90.0 <= latitude <= 90.0
            or not -180.0 <= longitude <= 180.0
        ):
            continue
        result[identifier] = {
            'intersection_id': identifier,
            'name': str(record.get('itstNm', record.get('name', ''))).strip(),
            'latitude': latitude,
            'longitude': longitude,
        }
    return list(result.values())


def load_intersection_map(path: Path) -> List[Intersection]:
    document = json.loads(path.read_text(encoding='utf-8'))
    intersections = normalize_intersections(document)
    if not intersections:
        raise ValueError('cached intersection map contains no usable records')
    return intersections


def save_intersection_map(
    path: Path,
    intersections: Iterable[Mapping[str, Any]],
) -> None:
    normalized = normalize_intersections(list(intersections))
    if not normalized:
        raise ValueError('refusing to cache an empty intersection map')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(
            {'intersections': normalized},
            ensure_ascii=False,
            indent=2,
        ) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def select_intersection_id(
    *,
    locked_id: Any,
    crosswalk: Optional[Mapping[str, Any]],
    nearest: Optional[Mapping[str, Any]],
    configured_id: Any,
) -> tuple[str, str, str]:
    choices = (
        ('locked', locked_id, ''),
        (
            'crosswalk_data',
            (crosswalk or {}).get('intersection_id', ''),
            '',
        ),
        (
            'v2x_nearest',
            (nearest or {}).get('intersection_id', ''),
            (nearest or {}).get('name', ''),
        ),
        ('configured_fallback', configured_id, ''),
    )
    for source, identifier, name in choices:
        normalized = str(identifier or '').strip()
        if normalized:
            return normalized, source, str(name or '').strip()
    return '', 'none', ''


def request_intersection_map(
    api_key: str,
    *,
    url: str = DEFAULT_INTERSECTION_MAP_URL,
    page_size: int = 100,
    max_pages: int = 30,
    timeout_s: float = 3.0,
) -> List[Intersection]:
    if not api_key:
        raise ValueError('API key is empty')
    if page_size <= 0 or max_pages <= 0:
        raise ValueError('intersection map pagination must be positive')
    intersections: Dict[str, Intersection] = {}
    for page in range(1, max_pages + 1):
        query = urllib.parse.urlencode(
            {
                'apiKey': api_key,
                'type': 'json',
                'pageNo': page,
                'numOfRows': page_size,
            }
        )
        request = urllib.request.Request(
            url + '?' + query,
            headers={'User-Agent': 'safestride-crosswalk/1.0'},
        )
        try:
            with urllib.request.urlopen(
                request,
                timeout=timeout_s,
            ) as response:
                body = response.read().decode('utf-8-sig')
        except urllib.error.HTTPError as error:
            detail = error.read().decode('utf-8', errors='replace')
            raise RuntimeError(
                'intersection map API HTTP %s: %s'
                % (error.code, detail[:160])
            ) from error
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            raise RuntimeError(
                'intersection map API request failed: %s' % error
            ) from error
        try:
            document = json.loads(body)
        except json.JSONDecodeError as error:
            raise ValueError(
                'intersection map response is not JSON'
            ) from error
        raw_record_count = sum(1 for _record in _records(document))
        page_records = normalize_intersections(document)
        for item in page_records:
            intersections[item['intersection_id']] = item
        if raw_record_count < page_size:
            break
    if not intersections:
        raise ValueError('intersection map contains no usable records')
    return list(intersections.values())


def nearest_intersection(
    intersections: Iterable[Mapping[str, Any]],
    latitude: float,
    longitude: float,
    *,
    maximum_distance_m: float,
) -> Optional[Intersection]:
    if not math.isfinite(maximum_distance_m) or maximum_distance_m <= 0.0:
        raise ValueError('maximum_distance_m must be finite and positive')
    selected: Optional[Mapping[str, Any]] = None
    selected_distance = math.inf
    for item in intersections:
        distance = haversine_m(
            latitude,
            longitude,
            float(item['latitude']),
            float(item['longitude']),
        )
        if distance <= maximum_distance_m and distance < selected_distance:
            selected = item
            selected_distance = distance
    if selected is None:
        return None
    result = dict(selected)
    result['distance_m'] = selected_distance
    return result


__all__ = [
    'DEFAULT_INTERSECTION_MAP_URL',
    'load_intersection_map',
    'nearest_intersection',
    'normalize_intersections',
    'request_intersection_map',
    'save_intersection_map',
    'select_intersection_id',
]
