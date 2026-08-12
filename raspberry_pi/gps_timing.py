import csv
import json
import math
import urllib.error
import urllib.parse
import urllib.request

MAP_URL = "http://t-data.seoul.go.kr/apig/apiman-gateway/tapi/v2xCrossroadMapInformation/1.0"
TIMING_URL = "http://t-data.seoul.go.kr/apig/apiman-gateway/tapi/v2xSignalPhaseTimingInformation/1.0"


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


def request_rows(url, api_key, extra=None):
    params = {
        "apikey": api_key,
        "type": "json",
        "pageNo": 1,
        "numOfRows": 10000
    }

    if extra:
        params.update(extra)

    full_url = url + "?" + urllib.parse.urlencode(params)

    request = urllib.request.Request(
        full_url,
        headers={"User-Agent": "Mozilla/5.0"}
    )

    with urllib.request.urlopen(request, timeout=30) as response:
        text = response.read().decode("utf-8")

    data = json.loads(text)
    return collect_rows(data)
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

    return radius * 2.0 * math.atan2(
        math.sqrt(value),
        math.sqrt(1.0 - value)
    )


def read_position():
    with open("current_position.csv", newline="") as file:
        reader = csv.DictReader(file)
        row = next(reader)

    return float(row["latitude"]), float(row["longitude"])


def get_remaining_seconds(value):
    if value in (None, ""):
        return None

    try:
        value = float(value)
    except ValueError:
        return None

    if value < 0 or value >= 36000:
        return None

    return value / 10.0


timing_key = input("Timing API key: ").strip()
map_key = input("Map API key (Enter = same key): ").strip()

if not map_key:
    map_key = timing_key
try:
    current_lat, current_lon = read_position()

    print()
    print("Current GPS")
    print("Latitude:", current_lat)
    print("Longitude:", current_lon)

    print()
    print("Downloading crossroad map...")

    map_rows = request_rows(MAP_URL, map_key)

    print("Map rows:", len(map_rows))

    nearest = None

    for row in map_rows:
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
        print("No crossroad coordinate found")
        raise SystemExit

    print()
    print("Nearest crossroad")
    print("Name:", nearest["name"])
    print("itstId:", nearest["itstId"])
    print("Distance:", round(nearest["distance"], 1), "m")
    print("Latitude:", nearest["latitude"])
    print("Longitude:", nearest["longitude"])

    print()
    print("Downloading signal timing...")

    timing_rows = request_rows(
        TIMING_URL,
        timing_key,
        {
            "itstId": nearest["itstId"],
            "numOfRows": 100
        }
    )

    matches = [
        row for row in timing_rows
        if str(row.get("itstId", "")) == nearest["itstId"]
    ]

    if not matches:
        print("No timing data for itstId:", nearest["itstId"])
        raise SystemExit

    latest = matches[-1]

    directions = [
        ("nt", "North"),
        ("et", "East"),
        ("st", "South"),
        ("wt", "West"),
        ("ne", "North-East"),
        ("se", "South-East"),
        ("sw", "South-West"),
        ("nw", "North-West")
    ]

    print()
    print("Pedestrian remaining time")
    print("itstId:", latest.get("itstId", ""))

    found = False

    for code, name in directions:
        field = code + "PdsgRmdrCs"
        seconds = get_remaining_seconds(latest.get(field))

        if seconds is not None:
            found = True
            print(f"{name}: {seconds:.1f} seconds")

    if not found:
        print("No valid pedestrian remaining time")

except FileNotFoundError:
    print("current_position.csv does not exist")

except urllib.error.HTTPError as error:
    print("HTTP error:", error.code, error.reason)
    print("Check the API key and API approval")

except urllib.error.URLError as error:
    print("Network error:", error.reason)

except json.JSONDecodeError:
    print("API response is not JSON")

except Exception as error:
    print("Error:", error)