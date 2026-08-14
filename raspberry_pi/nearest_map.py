from pathlib import Path
import csv
import json
import math
import subprocess
import urllib.parse

MAP_URL = "http://t-data.seoul.go.kr/apig/apiman-gateway/tapi/v2xCrossroadMapInformation/1.0"

PAGE_SIZE = 100
MAX_PAGES = 30


def collect_rows(data):
    rows = []

    if isinstance(data, dict):
        if "itstId" in data:
            rows.append(data)

        for value in data.values():
            rows.extend(collect_rows(value))

    elif isinstance(data, list):
        for value in data:
            rows.extend(collect_rows(value))

    return rows


def fetch_page(api_key, page_number):
    params = {
        "apiKey": api_key,
        "type": "json",
        "pageNo": page_number,
        "numOfRows": PAGE_SIZE
    }

    url = MAP_URL + "?" + urllib.parse.urlencode(params)

    result = subprocess.run(
        ["curl", "-sS", "-L", "--http1.1", url],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        raise RuntimeError(result.stderr)

    data = json.loads(result.stdout)

    return collect_rows(data)
def read_position():
    with open("current_position.csv", newline="") as file:
        reader = csv.DictReader(file)
        row = next(reader)

    latitude = float(row["latitude"])
    longitude = float(row["longitude"])

    return latitude, longitude


def haversine(lat1, lon1, lat2, lon2):
    radius = 6371000.0

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    value = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(dlon / 2.0) ** 2
    )

    distance = radius * 2.0 * math.atan2(
        math.sqrt(value),
        math.sqrt(1.0 - value)
    )

    return distance


_key_file = Path(__file__).resolve().parent / "api_key.txt"
if not _key_file.exists():
    raise FileNotFoundError("api_key.txt not found")
api_key = _key_file.read_text(encoding="utf-8").strip()
if not api_key:
    raise ValueError("api_key.txt is empty")

try:
    current_lat, current_lon = read_position()

    print()
    print("Current GPS")
    print("Latitude:", current_lat)
    print("Longitude:", current_lon)

    crossroads = {}

    for page in range(1, MAX_PAGES + 1):
        rows = fetch_page(api_key, page)

        print("Page", page, "rows:", len(rows))

        if not rows:
            break

        for row in rows:
            itst_id = str(row.get("itstId", ""))

            if itst_id:
                crossroads[itst_id] = row

        if len(rows) < PAGE_SIZE:
            break

    print("Total crossroads:", len(crossroads))

    nearest = None
    for row in crossroads.values():
        try:
            cross_lat = float(row["mapCtptIntLat"])
            cross_lon = float(row["mapCtptIntLot"])

            distance = haversine(
                current_lat,
                current_lon,
                cross_lat,
                cross_lon
            )

            if nearest is None or distance < nearest["distance"]:
                nearest = {
                    "distance": distance,
                    "itstId": str(row.get("itstId", "")),
                    "name": str(row.get("itstNm", "")),
                    "latitude": cross_lat,
                    "longitude": cross_lon
                }

        except (KeyError, TypeError, ValueError):
            continue

    if nearest is None:
        print("No valid crossroad coordinate found")
        raise SystemExit

    print()
    print("-------------------------")
    print("Nearest crossroad")
    print("Name:", nearest["name"])
    print("itstId:", nearest["itstId"])
    print("Distance:", round(nearest["distance"], 1), "m")
    print("Latitude:", nearest["latitude"])
    print("Longitude:", nearest["longitude"])

    with open("nearest_crossroad.json", "w") as file:
        json.dump(nearest, file, indent=2)

    print()
    print("Saved: nearest_crossroad.json")

except FileNotFoundError:
    print("current_position.csv does not exist")

except json.JSONDecodeError:
    print("Map API response is not JSON")

except Exception as error:
    print("Error:", error)
