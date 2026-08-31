#!/usr/bin/env python3
import csv
import json
import math
import os
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed.")
    print("Run: sudo apt install -y python3-serial")
    sys.exit(1)

BASE_DIR = Path(__file__).resolve().parent

API_KEY_FILE = BASE_DIR / "api_key.txt"
CROSSWALK_FILE = BASE_DIR / "standard_crosswalks.json"
CURRENT_POSITION_FILE = BASE_DIR / "current_position.csv"
NEAREST_CROSSROAD_FILE = BASE_DIR / "nearest_crossroad.json"
NEAREST_MAP_SCRIPT = BASE_DIR / "nearest_map.py"
LOG_FILE = BASE_DIR / "crosswalk_live_log.csv"

TIMING_URL = (
    "https://t-data.seoul.go.kr/apig/apiman-gateway/"
    "tapi/v2xSignalPhaseTimingInformation/1.0"
)

GPS_PORT_CANDIDATES = (
    "/dev/serial0",
    "/dev/ttyAMA0",
    "/dev/ttyUSB0",
)
GPS_BAUD = 115200

UPDATE_INTERVAL_S = 1.0
MAP_REFRESH_INTERVAL_S = 60.0
MAP_REFRESH_DISTANCE_M = 25.0
MAX_CROSSWALK_DISTANCE_M = 60.0

WALKING_SPEED_MPS = 0.60
SAFETY_MARGIN_S = 3.0
HEADING_MIN_MOVE_M = 2.0

INVALID_SIGNAL_VALUES = {36000, 36001, -1}

DIRECTION_NAMES = {
    "nt": "N",
    "ne": "NE",
    "et": "E",
    "se": "SE",
    "st": "S",
    "sw": "SW",
    "wt": "W",
    "nw": "NW",
}

OPPOSITE_DIRECTION = {
    "nt": "st",
    "ne": "sw",
    "et": "wt",
    "se": "nw",
    "st": "nt",
    "sw": "ne",
    "wt": "et",
    "nw": "se",
}


def number(value):
    try:
        if value is None:
            return None
        text = str(value).strip().replace(",", "")
        if not text:
            return None
        return float(text)
    except (TypeError, ValueError):
        return None


def first_value(record, names):
    for name in names:
        value = record.get(name)
        if value not in (None, ""):
            return value
    return None


def find_value(data, key):
    if isinstance(data, dict):
        if key in data:
            return data[key]
        for value in data.values():
            result = find_value(value, key)
            if result is not None:
                return result
    elif isinstance(data, list):
        for value in data:
            result = find_value(value, key)
            if result is not None:
                return result
    return None


def collect_keys(data):
    keys = set()
    if isinstance(data, dict):
        for key, value in data.items():
            keys.add(str(key))
            keys.update(collect_keys(value))
    elif isinstance(data, list):
        for value in data:
            keys.update(collect_keys(value))
    return keys


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


def bearing_deg(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)

    y = math.sin(dl) * math.cos(p2)
    x = (
        math.cos(p1) * math.sin(p2)
        - math.sin(p1) * math.cos(p2) * math.cos(dl)
    )
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def bearing_to_direction(angle):
    directions = ("nt", "ne", "et", "se", "st", "sw", "wt", "nw")
    index = int((angle + 22.5) // 45.0) % 8
    return directions[index]


def nmea_coord_to_decimal(raw, hemisphere):
    if not raw:
        return None

    value = float(raw)
    degrees = int(value // 100)
    minutes = value - degrees * 100
    decimal = degrees + minutes / 60.0

    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_nmea(line):
    if not line.startswith("$"):
        return None

    payload = line.split("*", 1)[0]
    fields = payload.split(",")
    sentence = fields[0]

    try:
        if sentence in ("$GNRMC", "$GPRMC"):
            if len(fields) < 8 or fields[2] != "A":
                return None

            lat = nmea_coord_to_decimal(fields[3], fields[4])
            lon = nmea_coord_to_decimal(fields[5], fields[6])
            speed_knots = number(fields[7])
            course = number(fields[8]) if len(fields) > 8 else None

            if lat is None or lon is None:
                return None

            return {
                "latitude": lat,
                "longitude": lon,
                "gps_speed_mps": (
                    None if speed_knots is None else speed_knots * 0.514444
                ),
                "gps_course_deg": course,
                "source": sentence,
            }

        if sentence in ("$GNGGA", "$GPGGA"):
            if len(fields) < 7:
                return None

            fix_quality = int(fields[6] or "0")
            if fix_quality <= 0:
                return None

            lat = nmea_coord_to_decimal(fields[2], fields[3])
            lon = nmea_coord_to_decimal(fields[4], fields[5])

            if lat is None or lon is None:
                return None

            return {
                "latitude": lat,
                "longitude": lon,
                "gps_speed_mps": None,
                "gps_course_deg": None,
                "source": sentence,
            }
    except (ValueError, IndexError):
        return None

    return None


def open_gps():
    errors = []

    for port in GPS_PORT_CANDIDATES:
        if not os.path.exists(port):
            continue

        try:
            device = serial.Serial(
                port=port,
                baudrate=GPS_BAUD,
                timeout=1.2,
            )
            device.reset_input_buffer()
            return device, port
        except Exception as error:
            errors.append("%s: %s" % (port, error))

    detail = "; ".join(errors) if errors else "No GPS serial port found"
    raise RuntimeError(detail)


def read_gps_fix(device, max_wait_s=3.0):
    deadline = time.monotonic() + max_wait_s

    while time.monotonic() < deadline:
        raw = device.readline()
        if not raw:
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        fix = parse_nmea(line)
        if fix is not None:
            return fix

    raise TimeoutError("No valid GPS fix")


def save_current_position(lat, lon):
    temp_file = CURRENT_POSITION_FILE.with_suffix(".tmp")

    with temp_file.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("latitude", "longitude"))
        writer.writerow(("%.8f" % lat, "%.8f" % lon))

    temp_file.replace(CURRENT_POSITION_FILE)


def load_crosswalks():
    if not CROSSWALK_FILE.exists():
        raise FileNotFoundError("standard_crosswalks.json not found")

    with CROSSWALK_FILE.open(encoding="utf-8-sig") as file:
        data = json.load(file)

    if isinstance(data, dict):
        rows = None
        for key in ("data", "items", "row", "list"):
            candidate = data.get(key)
            if isinstance(candidate, list):
                rows = candidate
                break
        if rows is None:
            raise ValueError("Crosswalk JSON list not found")
    elif isinstance(data, list):
        rows = data
    else:
        raise ValueError("Invalid crosswalk JSON")

    result = []

    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            continue

        lat = number(first_value(row, ("latitude", "lat", "y")))
        lon = number(first_value(row, ("longitude", "lon", "lng", "x")))
        length = number(
            first_value(row, ("length_m", "crswlkLt", "et", "length"))
        )
        width = number(
            first_value(row, ("width_m", "crswlkBt", "bt", "width"))
        )

        if lat is None or lon is None or length is None or length <= 0:
            continue

        result.append(
            {
                "index": index,
                "latitude": lat,
                "longitude": lon,
                "length_m": length,
                "width_m": width,
                "raw": row,
            }
        )

    if not result:
        raise ValueError("No valid crosswalk records")

    return result


def nearest_crosswalk(crosswalks, lat, lon):
    best = None
    best_distance = None

    for item in crosswalks:
        distance = haversine_m(
            lat,
            lon,
            item["latitude"],
            item["longitude"],
        )

        if best_distance is None or distance < best_distance:
            best = item
            best_distance = distance

    output = dict(best)
    output["distance_m"] = best_distance
    return output


def load_itst_id():
    if not NEAREST_CROSSROAD_FILE.exists():
        return None

    try:
        with NEAREST_CROSSROAD_FILE.open(encoding="utf-8-sig") as file:
            data = json.load(file)
    except Exception:
        return None

    value = find_value(data, "itstId")
    if value in (None, ""):
        return None
    return str(value)


def refresh_nearest_crossroad():
    if not NEAREST_MAP_SCRIPT.exists():
        raise FileNotFoundError("nearest_map.py not found")

    result = subprocess.run(
        [sys.executable, str(NEAREST_MAP_SCRIPT)],
        cwd=str(BASE_DIR),
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        timeout=60,
    )

    if result.returncode != 0:
        detail = result.stderr.strip()
        raise RuntimeError(detail or "nearest_map.py failed")

    itst_id = load_itst_id()
    if not itst_id:
        raise ValueError("itstId not found")
    return itst_id


def read_api_key():
    if not API_KEY_FILE.exists():
        raise FileNotFoundError("api_key.txt not found")

    key = API_KEY_FILE.read_text(encoding="utf-8-sig").strip()
    if not key:
        raise ValueError("api_key.txt is empty")
    return key


def request_signal_data(api_key, itst_id):
    query = urllib.parse.urlencode(
        {
            "apikey": api_key,
            "itstId": itst_id,
            "type": "json",
            "pageNo": 1,
            "numOfRows": 100,
        }
    )

    request = urllib.request.Request(
        TIMING_URL + "?" + query,
        headers={"User-Agent": "smart-crosswalk-live/1.0"},
    )

    try:
        with urllib.request.urlopen(request, timeout=2.5) as response:
            body = response.read().decode("utf-8-sig")
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise RuntimeError("HTTP %s: %s" % (error.code, detail[:120]))
    except urllib.error.URLError as error:
        raise RuntimeError("Network: %s" % error.reason)

    try:
        return json.loads(body)
    except json.JSONDecodeError:
        raise ValueError("Signal response is not JSON")


def signal_remaining(data, direction):
    candidates = (direction, OPPOSITE_DIRECTION[direction])
    errors = []

    for candidate in candidates:
        field = candidate + "PdsgRmdrCs"
        raw = find_value(data, field)

        if raw in (None, ""):
            errors.append(field + " missing")
            continue

        try:
            value = int(float(raw))
        except (TypeError, ValueError):
            errors.append(field + " invalid")
            continue

        if value in INVALID_SIGNAL_VALUES or value < 0:
            errors.append(field + "=" + str(value))
            continue

        return value / 10.0, field

    available = sorted(
        key for key in collect_keys(data)
        if key.endswith("PdsgRmdrCs")
    )

    raise ValueError(
        "; ".join(errors)
        + "; available="
        + ",".join(available)
    )


def append_log(row):
    exists = LOG_FILE.exists() and LOG_FILE.stat().st_size > 0

    with LOG_FILE.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def clear_screen():
    print("\033[2J\033[H", end="")


def main():
    api_key = read_api_key()
    crosswalks = load_crosswalks()
    gps, gps_port = open_gps()

    previous_fix = None
    last_direction = None
    last_map_position = None
    last_map_update = 0.0
    itst_id = None
    last_log_time = 0.0

    try:
        while True:
            cycle_start = time.monotonic()
            error_text = ""
            signal_s = None
            signal_field = ""
            decision = "NO SIGNAL DATA"

            try:
                fix = read_gps_fix(gps)
                lat = fix["latitude"]
                lon = fix["longitude"]
                save_current_position(lat, lon)

                crosswalk = nearest_crosswalk(crosswalks, lat, lon)

                if previous_fix is not None:
                    moved = haversine_m(
                        previous_fix["latitude"],
                        previous_fix["longitude"],
                        lat,
                        lon,
                    )
                    if moved >= HEADING_MIN_MOVE_M:
                        angle = bearing_deg(
                            previous_fix["latitude"],
                            previous_fix["longitude"],
                            lat,
                            lon,
                        )
                        last_direction = bearing_to_direction(angle)

                if last_direction is None:
                    angle = bearing_deg(
                        lat,
                        lon,
                        crosswalk["latitude"],
                        crosswalk["longitude"],
                    )
                    last_direction = bearing_to_direction(angle)

                now = time.monotonic()
                map_distance = None

                if last_map_position is not None:
                    map_distance = haversine_m(
                        last_map_position[0],
                        last_map_position[1],
                        lat,
                        lon,
                    )

                need_map_refresh = (
                    itst_id is None
                    or now - last_map_update >= MAP_REFRESH_INTERVAL_S
                    or (
                        map_distance is not None
                        and map_distance >= MAP_REFRESH_DISTANCE_M
                    )
                )

                if need_map_refresh:
                    itst_id = refresh_nearest_crossroad()
                    last_map_position = (lat, lon)
                    last_map_update = time.monotonic()

                required_time = (
                    crosswalk["length_m"] / WALKING_SPEED_MPS
                    + SAFETY_MARGIN_S
                )

                if crosswalk["distance_m"] <= MAX_CROSSWALK_DISTANCE_M:
                    data = request_signal_data(api_key, itst_id)
                    signal_s, signal_field = signal_remaining(
                        data,
                        last_direction,
                    )

                    if signal_s >= required_time:
                        decision = "CAN CROSS"
                    else:
                        decision = "WAIT"
                else:
                    error_text = "Crosswalk is farther than %.0f m" % (
                        MAX_CROSSWALK_DISTANCE_M
                    )

                previous_fix = fix

            except Exception as error:
                error_text = str(error)

            clear_screen()
            print("SMART CROSSWALK LIVE")
            print("====================")
            print("GPS port       :", gps_port)
            print("Refresh        : 1 second")
            print("Time           :", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

            if "fix" in locals():
                print("Latitude       : %.8f" % fix["latitude"])
                print("Longitude      : %.8f" % fix["longitude"])
            else:
                print("Latitude       : -")
                print("Longitude      : -")

            if "crosswalk" in locals():
                print("Crosswalk dist : %.1f m" % crosswalk["distance_m"])
                print("Crosswalk len  : %.1f m" % crosswalk["length_m"])
                if crosswalk["width_m"] is not None:
                    print("Crosswalk width: %.1f m" % crosswalk["width_m"])
                else:
                    print("Crosswalk width: -")
            else:
                print("Crosswalk dist : -")
                print("Crosswalk len  : -")
                print("Crosswalk width: -")

            print("Intersection ID:", itst_id or "-")
            print(
                "Direction       :",
                DIRECTION_NAMES.get(last_direction, "-"),
                "(" + str(last_direction or "-") + ")",
            )
            print("Walking speed  : %.2f m/s" % WALKING_SPEED_MPS)
            print("Safety margin  : %.1f s" % SAFETY_MARGIN_S)

            if "required_time" in locals():
                print("Required time  : %.1f s" % required_time)
            else:
                print("Required time  : -")

            if signal_s is not None:
                print("Signal field   :", signal_field)
                print("Signal remains : %.1f s" % signal_s)
            else:
                print("Signal field   : -")
                print("Signal remains : -")

            print()
            print("RESULT         :", decision)

            if error_text:
                print("STATUS         :", error_text)
            else:
                print("STATUS         : OK")

            print()
            print("Press Ctrl+C to stop.")

            now_wall = time.monotonic()
            if (
                "fix" in locals()
                and "crosswalk" in locals()
                and now_wall - last_log_time >= 5.0
            ):
                append_log(
                    {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "latitude": "%.8f" % fix["latitude"],
                        "longitude": "%.8f" % fix["longitude"],
                        "itstId": itst_id or "",
                        "direction": last_direction or "",
                        "crosswalk_distance_m": round(
                            crosswalk["distance_m"], 2
                        ),
                        "crosswalk_length_m": round(
                            crosswalk["length_m"], 2
                        ),
                        "walking_speed_mps": WALKING_SPEED_MPS,
                        "required_time_s": (
                            round(required_time, 1)
                            if "required_time" in locals()
                            else ""
                        ),
                        "signal_remaining_s": (
                            round(signal_s, 1)
                            if signal_s is not None
                            else ""
                        ),
                        "decision": decision,
                        "status": error_text or "OK",
                    }
                )
                last_log_time = now_wall

            elapsed = time.monotonic() - cycle_start
            time.sleep(max(0.0, UPDATE_INTERVAL_S - elapsed))

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        gps.close()


if __name__ == "__main__":
    main()
