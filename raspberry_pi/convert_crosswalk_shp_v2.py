#!/usr/bin/env python3
import csv
import json
import math
import shutil
import sys
from datetime import datetime
from pathlib import Path

try:
    import shapefile
except ImportError:
    print("ERROR: python3-shapefile is not installed")
    print("Run: sudo apt install -y python3-shapefile")
    sys.exit(1)

try:
    from pyproj import CRS, Transformer
except ImportError:
    print("ERROR: python3-pyproj is not installed")
    print("Run: sudo apt install -y python3-pyproj")
    sys.exit(1)

BASE = Path(__file__).resolve().parent
DEFAULT_SHP = BASE / "crosswalk_shp" / "A004_A.shp"
OUTPUT = BASE / "standard_crosswalks.json"
POSITION = BASE / "current_position.csv"

SEOUL_LAT_MIN = 37.35
SEOUL_LAT_MAX = 37.75
SEOUL_LON_MIN = 126.70
SEOUL_LON_MAX = 127.30


def detect_encoding(shp_path):
    cpg_path = shp_path.with_suffix(".cpg")
    if not cpg_path.exists():
        return "cp949"

    value = cpg_path.read_text(encoding="ascii", errors="ignore").strip().upper()

    aliases = {
        "949": "cp949",
        "CP949": "cp949",
        "EUC-KR": "cp949",
        "EUC_KR": "cp949",
        "UTF-8": "utf-8",
        "UTF8": "utf-8",
        "65001": "utf-8",
    }
    return aliases.get(value, "cp949")


def load_transformer(shp_path):
    prj_path = shp_path.with_suffix(".prj")

    if prj_path.exists():
        wkt = prj_path.read_text(encoding="utf-8", errors="ignore").strip()
        if wkt:
            try:
                source_crs = CRS.from_wkt(wkt)
                return Transformer.from_crs(
                    source_crs,
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


def pca_dimensions(points):
    clean = []
    last = None

    for x, y in points:
        current = (float(x), float(y))
        if current != last:
            clean.append(current)
        last = current

    if len(clean) < 3:
        return None, None, None

    mean_x = sum(p[0] for p in clean) / len(clean)
    mean_y = sum(p[1] for p in clean) / len(clean)

    cov_xx = sum((p[0] - mean_x) ** 2 for p in clean) / len(clean)
    cov_yy = sum((p[1] - mean_y) ** 2 for p in clean) / len(clean)
    cov_xy = sum(
        (p[0] - mean_x) * (p[1] - mean_y)
        for p in clean
    ) / len(clean)

    angle = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    axis1 = []
    axis2 = []

    for x, y in clean:
        dx = x - mean_x
        dy = y - mean_y

        axis1.append(dx * cos_a + dy * sin_a)
        axis2.append(-dx * sin_a + dy * cos_a)

    span1 = max(axis1) - min(axis1)
    span2 = max(axis2) - min(axis2)

    if span1 >= span2:
        length = span1
        width = span2
        major_angle = angle
    else:
        length = span2
        width = span1
        major_angle = angle + math.pi / 2.0

    if length <= 0 or width <= 0:
        return None, None, None

    # Projected CRS: +X is east and +Y is north.
    # Convert mathematical angle from +X CCW to compass bearing from north CW.
    bearing = (90.0 - math.degrees(major_angle)) % 180.0

    return length, width, bearing


def polygon_centroid(points):
    clean = []
    last = None

    for x, y in points:
        current = (float(x), float(y))
        if current != last:
            clean.append(current)
        last = current

    if len(clean) < 3:
        return None

    if clean[0] != clean[-1]:
        clean.append(clean[0])

    area2 = 0.0
    cx_sum = 0.0
    cy_sum = 0.0

    for i in range(len(clean) - 1):
        x1, y1 = clean[i]
        x2, y2 = clean[i + 1]

        cross = x1 * y2 - x2 * y1
        area2 += cross
        cx_sum += (x1 + x2) * cross
        cy_sum += (y1 + y2) * cross

    if abs(area2) < 1e-9:
        xs = [p[0] for p in clean[:-1]]
        ys = [p[1] for p in clean[:-1]]
        return sum(xs) / len(xs), sum(ys) / len(ys)

    centroid_x = cx_sum / (3.0 * area2)
    centroid_y = cy_sum / (3.0 * area2)
    return centroid_x, centroid_y


def haversine_m(lat1, lon1, lat2, lon2):
    radius = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = (
        math.sin(dp / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    )

    return 2.0 * radius * math.asin(math.sqrt(a))


def load_current_position():
    if not POSITION.exists():
        return None

    with POSITION.open("r", newline="", encoding="utf-8-sig") as file:
        row = next(csv.DictReader(file), None)

    if not row:
        return None

    try:
        return float(row["latitude"]), float(row["longitude"])
    except (KeyError, TypeError, ValueError):
        return None


def main():
    shp_path = (
        Path(sys.argv[1]).expanduser().resolve()
        if len(sys.argv) >= 2
        else DEFAULT_SHP
    )

    if not shp_path.exists():
        print("ERROR: shapefile not found:", shp_path)
        sys.exit(2)

    required = (
        shp_path,
        shp_path.with_suffix(".dbf"),
        shp_path.with_suffix(".shx"),
        shp_path.with_suffix(".prj"),
    )

    for path in required:
        if not path.exists():
            print("ERROR: required shapefile component missing:", path)
            sys.exit(3)

    encoding = detect_encoding(shp_path)
    transformer = load_transformer(shp_path)

    reader = shapefile.Reader(
        str(shp_path),
        encoding=encoding,
        encodingErrors="replace",
    )

    field_names = [item[0] for item in reader.fields[1:]]

    rows = []
    skipped_geometry = 0
    skipped_outside = 0
    skipped_dimension = 0

    for index, shape_record in enumerate(reader.iterShapeRecords()):
        shape = shape_record.shape
        points = shape.points

        if not points or len(points) < 3:
            skipped_geometry += 1
            continue

        centroid = polygon_centroid(points)
        if centroid is None:
            skipped_geometry += 1
            continue

        source_x, source_y = centroid
        lon, lat = transformer.transform(source_x, source_y)

        if not (
            SEOUL_LAT_MIN <= lat <= SEOUL_LAT_MAX
            and SEOUL_LON_MIN <= lon <= SEOUL_LON_MAX
        ):
            skipped_outside += 1
            continue

        length, width, axis_bearing = pca_dimensions(points)

        if (
            length is None
            or width is None
            or axis_bearing is None
            or length <= 0
            or width <= 0
        ):
            skipped_dimension += 1
            continue

        rows.append(
            {
                "latitude": round(lat, 8),
                "longitude": round(lon, 8),
                "et": round(length, 2),
                "bt": round(width, 2),
                "axis_bearing_deg": round(axis_bearing, 1),
                "source_index": index,
            }
        )

    reader.close()

    if not rows:
        print("ERROR: no valid crosswalk polygons were converted")
        sys.exit(4)

    unique = {}

    for row in rows:
        key = (
            round(row["latitude"], 7),
            round(row["longitude"], 7),
        )
        unique[key] = row

    rows = list(unique.values())

    if OUTPUT.exists():
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = BASE / f"standard_crosswalks_backup_{stamp}.json"
        shutil.copy2(OUTPUT, backup)
        print("Backup:", backup.name)

    temp = OUTPUT.with_suffix(".tmp")

    with temp.open("w", encoding="utf-8") as file:
        json.dump(rows, file, ensure_ascii=False, indent=2)

    temp.replace(OUTPUT)

    latitudes = [row["latitude"] for row in rows]
    longitudes = [row["longitude"] for row in rows]

    print("Input:", shp_path)
    print("Encoding:", encoding)
    print("DBF fields:", field_names)
    print("Saved:", OUTPUT)
    print("Valid crosswalks:", len(rows))
    print("Skipped geometry:", skipped_geometry)
    print("Skipped dimensions:", skipped_dimension)
    print("Skipped outside Seoul:", skipped_outside)
    print("Latitude range:", min(latitudes), max(latitudes))
    print("Longitude range:", min(longitudes), max(longitudes))

    current = load_current_position()

    if current:
        user_lat, user_lon = current

        nearest = min(
            rows,
            key=lambda row: haversine_m(
                user_lat,
                user_lon,
                row["latitude"],
                row["longitude"],
            ),
        )

        distance = haversine_m(
            user_lat,
            user_lon,
            nearest["latitude"],
            nearest["longitude"],
        )

        print("Current GPS:", user_lat, user_lon)
        print("Nearest distance:", round(distance, 1), "m")
        print("Nearest length:", nearest["et"], "m")
        print("Nearest width:", nearest["bt"], "m")


if __name__ == "__main__":
    main()
