#!/usr/bin/env python3
"""Convert the supplied Seoul A004 crosswalk polygons to runtime JSON."""

import argparse
import json
import math
from pathlib import Path

try:
    import shapefile
    from pyproj import CRS, Transformer
except ImportError as error:
    raise SystemExit(
        'Install converter dependencies: '
        'sudo apt install python3-shapefile python3-pyproj'
    ) from error


SEOUL_BOUNDS = (37.35, 37.75, 126.70, 127.30)


def detect_encoding(path: Path) -> str:
    cpg = path.with_suffix('.cpg')
    if not cpg.exists():
        return 'cp949'
    value = cpg.read_text(encoding='ascii', errors='ignore').strip().upper()
    return {
        '949': 'cp949',
        'CP949': 'cp949',
        'EUC-KR': 'cp949',
        'UTF-8': 'utf-8',
        'UTF8': 'utf-8',
        '65001': 'utf-8',
    }.get(value, 'cp949')


def transformer_for(path: Path) -> Transformer:
    prj = path.with_suffix('.prj')
    if prj.exists():
        try:
            source = CRS.from_wkt(
                prj.read_text(encoding='utf-8', errors='ignore')
            )
            return Transformer.from_crs(
                source,
                CRS.from_epsg(4326),
                always_xy=True,
            )
        except Exception:
            pass
    return Transformer.from_crs(
        CRS.from_epsg(5186),
        CRS.from_epsg(4326),
        always_xy=True,
    )


def clean_points(points):
    result = []
    for point in points:
        current = (float(point[0]), float(point[1]))
        if not result or current != result[-1]:
            result.append(current)
    return result


def polygon_centroid(points):
    points = clean_points(points)
    if len(points) < 3:
        return None
    if points[0] != points[-1]:
        points.append(points[0])
    area2 = 0.0
    x_sum = 0.0
    y_sum = 0.0
    for first, second in zip(points, points[1:]):
        cross = first[0] * second[1] - second[0] * first[1]
        area2 += cross
        x_sum += (first[0] + second[0]) * cross
        y_sum += (first[1] + second[1]) * cross
    if abs(area2) < 1.0e-9:
        unique = points[:-1]
        return (
            sum(point[0] for point in unique) / len(unique),
            sum(point[1] for point in unique) / len(unique),
        )
    return x_sum / (3.0 * area2), y_sum / (3.0 * area2)


def pca_dimensions(points):
    points = clean_points(points)
    if len(points) < 3:
        return None
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    covariance_xx = sum((point[0] - mean_x) ** 2 for point in points)
    covariance_yy = sum((point[1] - mean_y) ** 2 for point in points)
    covariance_xy = sum(
        (point[0] - mean_x) * (point[1] - mean_y) for point in points
    )
    angle = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy,
    )
    first_axis = []
    second_axis = []
    for x, y in points:
        dx, dy = x - mean_x, y - mean_y
        first_axis.append(dx * math.cos(angle) + dy * math.sin(angle))
        second_axis.append(-dx * math.sin(angle) + dy * math.cos(angle))
    first_span = max(first_axis) - min(first_axis)
    second_span = max(second_axis) - min(second_axis)
    if first_span >= second_span:
        length, width, major_angle = first_span, second_span, angle
    else:
        length, width, major_angle = (
            second_span,
            first_span,
            angle + math.pi / 2.0,
        )
    if length <= 0.0 or width <= 0.0:
        return None
    bearing = (90.0 - math.degrees(major_angle)) % 180.0
    return length, width, bearing


def convert(source: Path):
    for suffix in ('.shp', '.dbf', '.shx', '.prj'):
        required = source.with_suffix(suffix)
        if not required.exists():
            raise FileNotFoundError(str(required))
    transform = transformer_for(source)
    reader = shapefile.Reader(
        str(source),
        encoding=detect_encoding(source),
        encodingErrors='replace',
    )
    result = {}
    latitude_min, latitude_max, longitude_min, longitude_max = SEOUL_BOUNDS
    try:
        for index, shape_record in enumerate(reader.iterShapeRecords()):
            centroid = polygon_centroid(shape_record.shape.points)
            dimensions = pca_dimensions(shape_record.shape.points)
            if centroid is None or dimensions is None:
                continue
            longitude, latitude = transform.transform(*centroid)
            if not (
                latitude_min <= latitude <= latitude_max
                and longitude_min <= longitude <= longitude_max
            ):
                continue
            length, width, bearing = dimensions
            record = {
                'latitude': round(latitude, 8),
                'longitude': round(longitude, 8),
                'length_m': round(length, 2),
                'width_m': round(width, 2),
                'axis_bearing_deg': round(bearing, 1),
                'source_index': index,
            }
            key = (round(latitude, 7), round(longitude, 7))
            result[key] = record
    finally:
        reader.close()
    if not result:
        raise ValueError('no valid Seoul crosswalk polygons were converted')
    return list(result.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path, help='Path to A004_A.shp')
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('data/generated/standard_crosswalks.json'),
    )
    arguments = parser.parse_args()
    records = convert(arguments.source.expanduser().resolve())
    output = arguments.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + '.tmp')
    temporary.write_text(
        json.dumps(records, ensure_ascii=False, indent=2),
        encoding='utf-8',
    )
    temporary.replace(output)
    print('Saved %d crosswalks to %s' % (len(records), output))


if __name__ == '__main__':
    main()
