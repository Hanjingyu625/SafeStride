import csv
import json
import math
import re

R = 6371000.0


def read_position():
    with open("current_position.csv", newline="") as file:
        row = next(csv.DictReader(file))

    return float(row["latitude"]), float(row["longitude"])


def parse_point(wkt):
    if not wkt:
        return None

    match = re.search(
        r"POINT\s*\(\s*([-\d.]+)\s+([-\d.]+)\s*\)",
        wkt,
        re.I
    )

    if not match:
        return None

    longitude = float(match.group(1))
    latitude = float(match.group(2))

    return latitude, longitude


def distance_m(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    value = (
        math.sin(dp / 2) ** 2
        + math.cos(p1)
        * math.cos(p2)
        * math.sin(dl / 2) ** 2
    )

    return R * 2 * math.atan2(
        math.sqrt(value),
        math.sqrt(1 - value)
    )


current_lat, current_lon = read_position()

with open("crosswalks.json") as file:
    rows = json.load(file)

results = []

for row in rows:
    point = parse_point(row.get("NODE_WKT", ""))

    if point is None:
        continue

    latitude, longitude = point

    distance = distance_m(
        current_lat,
        current_lon,
        latitude,
        longitude
    )

    results.append({
        "node_id": str(row.get("NODE_ID", "")),
        "distance_m": distance,
        "latitude": latitude,
        "longitude": longitude,
        "district": str(row.get("SGG_NM", "")),
        "dong": str(row.get("EMD_NM", "")),
        "width_m": None
    })

results.sort(key=lambda item: item["distance_m"])

print("Current GPS:", current_lat, current_lon)
print()
print("Nearest crosswalk points")

for index, item in enumerate(results[:5], start=1):
    print(
        index,
        "|",
        round(item["distance_m"], 1),
        "m |",
        item["district"],
        item["dong"],
        "| NODE_ID:",
        item["node_id"]
    )

if results:
    with open("nearest_crosswalk.json", "w") as file:
        json.dump(
            results[0],
            file,
            ensure_ascii=False,
            indent=2
        )

    print()
    print("Saved: nearest_crosswalk.json")
else:
    print("No NODE_WKT data found")

