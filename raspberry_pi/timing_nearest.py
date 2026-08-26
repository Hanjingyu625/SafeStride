import json
import subprocess
import urllib.parse

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


def fetch_timing(api_key, itst_id):
    params = {
        "apiKey": api_key,
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

    data = json.loads(result.stdout)

    return collect_rows(data)


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
api_key = input("Timing API key: ").strip()

try:
    with open("nearest_crossroad.json") as file:
        nearest = json.load(file)

    itst_id = str(nearest["itstId"])

    print()
    print("Nearest crossroad")
    print("Name:", nearest.get("name", ""))
    print("itstId:", itst_id)
    print("Distance:", round(nearest.get("distance", 0), 1), "m")

    print()
    print("Downloading signal timing...")

    rows = fetch_timing(api_key, itst_id)

    matches = [
        row for row in rows
        if str(row.get("itstId", "")) == itst_id
    ]

    if not matches:
        print("No timing data for itstId:", itst_id)
        raise SystemExit

    def time_value(row):
        value = row.get("trsmUtcTime") or row.get("trsmTm") or ""

        try:
            return float(value)
        except (TypeError, ValueError):
            return str(value)

    latest = max(matches, key=time_value)

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
    print("-------------------------")
    print("Pedestrian remaining time")
    print("Transmission time:", latest.get("trsmTm", ""))

    found = False

    for code, name in directions:
        field = code + "PdsgRmdrCs"
        seconds = convert_seconds(latest.get(field))

        if seconds is not None:
            found = True
            print(f"{name}: {seconds:.1f} seconds")

    if not found:
        print("No valid pedestrian remaining time")

except FileNotFoundError:
    print("nearest_crossroad.json does not exist")

except json.JSONDecodeError:
    print("API response is not JSON")

except Exception as error:
    print("Error:", error)

