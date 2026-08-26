import json
import math
import subprocess
import urllib.parse

PORT = "/dev/serial0"

TIMING_URL = (
    "http://t-data.seoul.go.kr/apig/apiman-gateway/"
    "tapi/v2xSignalPhaseTimingInformation/1.0"
)

DIRECTION_NAMES = {
    "nt": "North",
    "ne": "North-East",
    "et": "East",
    "se": "South-East",
    "st": "South",
    "sw": "South-West",
    "wt": "West",
    "nw": "North-West"
}


def nmea_to_decimal(value, hemisphere):
    degree_length = 2 if hemisphere in ("N", "S") else 3

    degrees = float(value[:degree_length])
    minutes = float(value[degree_length:])

    coordinate = degrees + minutes / 60.0

    if hemisphere in ("S", "W"):
        coordinate = -coordinate

    return coordinate


def read_fix(gps):
    while True:
        line = gps.readline().decode(
            "ascii",
            errors="ignore"
        ).strip()

        if not line.startswith(("$GNGGA", "$GPGGA")):
            continue

        parts = line.split(",")

        if len(parts) < 10:
            continue

        if parts[6] == "0":
            continue

        if not parts[2] or not parts[4]:
            continue

        latitude = nmea_to_decimal(
            parts[2],
            parts[3]
        )

        longitude = nmea_to_decimal(
            parts[4],
            parts[5]
        )

        return latitude, longitude


def average_fix(gps, count=5):
    latitudes = []
    longitudes = []

    while len(latitudes) < count:
        latitude, longitude = read_fix(gps)

        latitudes.append(latitude)
        longitudes.append(longitude)

    return (
        sum(latitudes) / len(latitudes),
        sum(longitudes) / len(longitudes)
    )
def distance_m(lat1, lon1, lat2, lon2):
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


def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    dlon = math.radians(lon2 - lon1)

    x = math.sin(dlon) * math.cos(lat2)

    y = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1)
        * math.cos(lat2)
        * math.cos(dlon)
    )

    angle = math.degrees(math.atan2(x, y))

    return (angle + 360.0) % 360.0


def bearing_to_direction(angle):
    directions = [
        "nt",
        "ne",
        "et",
        "se",
        "st",
        "sw",
        "wt",
        "nw"
    ]

    index = int((angle + 22.5) // 45.0) % 8

    return directions[index]


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


def fetch_timing(api_key, itst_id):
    params = {
        "apikey": api_key,
        "type": "json",
        "pageNo": 1,
        "numOfRows": 100,
        "itstId": itst_id
    }

    url = TIMING_URL + "?" + urllib.parse.urlencode(params)

    result = subprocess.run(
        ["curl", "-sS", "-L", "--http1.1", url],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        raise RuntimeError(result.stderr)

    return collect_rows(json.loads(result.stdout))


def convert_seconds(value):
    if value in (None, ""):
        return None

    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if value < 0 or value >= 36000:
        return None

    return value / 10.0
try:
    with open("api_key.txt") as file:
        api_key = file.read().strip()

    with open("nearest_crossroad.json") as file:
        crossroad = json.load(file)

    crosswalk_width = float(
        input("Crosswalk width (m): ")
    )

    walking_speed = float(
        input("Walking speed test value (m/s): ")
    )

    itst_id = str(crossroad["itstId"])

    print()
    print("Crossroad:", crossroad.get("name", ""))
    print("itstId:", itst_id)

    print()
    print("Getting first GPS position...")

    with open(PORT, "rb", buffering=0) as gps:
        start_lat, start_lon = average_fix(gps)

        print("Start position received")
        print("Move straight at least 5 meters")

        while True:
            end_lat, end_lon = average_fix(gps, 3)

            moved_distance = distance_m(
                start_lat,
                start_lon,
                end_lat,
                end_lon
            )

            print(
                "Moved:",
                round(moved_distance, 1),
                "m"
            )

            if moved_distance >= 5.0:
                break

    bearing = calculate_bearing(
        start_lat,
        start_lon,
        end_lat,
        end_lon
    )

    direction = bearing_to_direction(bearing)

    print()
    print("Movement bearing:", round(bearing, 1))
    print(
        "Movement direction:",
        DIRECTION_NAMES[direction],
        "(" + direction + ")"
    )

    print()
    print("Downloading signal timing...")

    rows = fetch_timing(api_key, itst_id)

    matches = [
        row for row in rows
        if str(row.get("itstId", "")) == itst_id
    ]

    if not matches:
        print("No timing data")
        raise SystemExit

    latest = matches[-1]

    available = {}

    for code in DIRECTION_NAMES:
        field = code + "PdsgRmdrCs"
        seconds = convert_seconds(latest.get(field))

        if seconds is not None:
            available[code] = seconds

    print()
    print("Available pedestrian signals")

    for code, seconds in available.items():
        print(
            DIRECTION_NAMES[code],
            ":",
            round(seconds, 1),
            "sec"
        )

    if direction not in available:
        print()
        print("No signal data for:", direction)
        raise SystemExit

    remaining_time = available[direction]

    safety_margin = 5.0
    crossing_time = crosswalk_width / walking_speed
    required_time = crossing_time + safety_margin

    print()
    print("-------------------------")
    print(
        "Selected direction:",
        DIRECTION_NAMES[direction]
    )
    print(
        "Remaining time:",
        round(remaining_time, 1),
        "sec"
    )
    print(
        "Crossing time:",
        round(crossing_time, 1),
        "sec"
    )
    print(
        "Required time:",
        round(required_time, 1),
        "sec"
    )

    if remaining_time >= required_time:
        print("Result: CAN CROSS")
    else:
        print("Result: WAIT")

except KeyboardInterrupt:
    print()
    print("Stopped")

except FileNotFoundError as error:
    print("Missing file:", error.filename)

except Exception as error:
    print("Error:", error)
