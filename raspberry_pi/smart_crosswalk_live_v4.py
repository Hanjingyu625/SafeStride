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
SIGNAL_REFRESH_INTERVAL_S = 3.0
SIGNAL_TIMEOUT_S = 10.0
SIGNAL_RETRIES = 3
SIGNAL_CACHE_MAX_AGE_S = 12.0
MAP_REFRESH_INTERVAL_S = 60.0
MAP_REFRESH_DISTANCE_M = 25.0
MAX_CROSSWALK_DISTANCE_M = 80.0
CROSSING_READY_DISTANCE_M = 20.0
MAX_AXIS_ALIGNMENT_ERROR_DEG = 50.0

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
        if sentence.endswith("RMC"):
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

        if sentence.endswith("GGA"):
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


def read_gps_fix(device, max_wait_s=10.0):
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
        axis_bearing = number(
            first_value(
                row,
                (
                    "axis_bearing_deg",
                    "axis_bearing",
                    "bearing_deg",
                ),
            )
        )

        if (
            lat is None
            or lon is None
            or length is None
            or length <= 0
            or axis_bearing is None
        ):
            continue

        result.append(
            {
                "index": index,
                "latitude": lat,
                "longitude": lon,
                "length_m": length,
                "width_m": width,
                "axis_bearing_deg": axis_bearing % 180.0,
                "raw": row,
            }
        )

    if not result:
        raise ValueError("No valid crosswalk records")

    return result


def local_offset_m(origin_lat, origin_lon, target_lat, target_lon):
    north = (target_lat - origin_lat) * 111320.0
    east = (
        (target_lon - origin_lon)
        * 111320.0
        * math.cos(math.radians((origin_lat + target_lat) / 2.0))
    )
    return east, north


def angular_difference_deg(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def undirected_axis_difference_deg(a, b):
    diff = angular_difference_deg(a, b)
    return min(diff, abs(180.0 - diff))


def crosswalk_edge_distance_m(item, user_lat, user_lon):
    east, north = local_offset_m(
        item["latitude"],
        item["longitude"],
        user_lat,
        user_lon,
    )

    bearing = math.radians(item["axis_bearing_deg"])
    axis_east = math.sin(bearing)
    axis_north = math.cos(bearing)

    side_east = math.sin(bearing + math.pi / 2.0)
    side_north = math.cos(bearing + math.pi / 2.0)

    along = east * axis_east + north * axis_north
    across = east * side_east + north * side_north

    half_length = max(item["length_m"], 0.0) / 2.0
    half_width = max(item["width_m"] or 0.0, 0.0) / 2.0

    outside_along = max(abs(along) - half_length, 0.0)
    outside_across = max(abs(across) - half_width, 0.0)

    return math.hypot(outside_along, outside_across)


def nearest_crosswalk(crosswalks, lat, lon):
    best = None
    best_edge_distance = None

    for item in crosswalks:
        edge_distance = crosswalk_edge_distance_m(item, lat, lon)

        if (
            best_edge_distance is None
            or edge_distance < best_edge_distance
        ):
            best = item
            best_edge_distance = edge_distance

    output = dict(best)
    output["edge_distance_m"] = best_edge_distance
    output["center_distance_m"] = haversine_m(
        lat,
        lon,
        best["latitude"],
        best["longitude"],
    )

    target_bearing = bearing_deg(
        lat,
        lon,
        best["latitude"],
        best["longitude"],
    )
    axis_a = best["axis_bearing_deg"] % 360.0
    axis_b = (axis_a + 180.0) % 360.0

    if angular_difference_deg(target_bearing, axis_a) <= angular_difference_deg(
        target_bearing,
        axis_b,
    ):
        crossing_bearing = axis_a
    else:
        crossing_bearing = axis_b

    output["target_bearing_deg"] = target_bearing
    output["crossing_bearing_deg"] = crossing_bearing
    output["crossing_direction"] = bearing_to_direction(crossing_bearing)
    output["axis_alignment_error_deg"] = undirected_axis_difference_deg(
        target_bearing,
        axis_a,
    )

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
            "apiKey": api_key,
            "itstId": itst_id,
            "type": "json",
            "pageNo": 1,
            "numOfRows": 100,
        }
    )

    request = urllib.request.Request(
        TIMING_URL + "?" + query,
        headers={"User-Agent": "smart-crosswalk-live/2.0"},
    )

    last_error = None

    for attempt in range(1, SIGNAL_RETRIES + 1):
        try:
            with urllib.request.urlopen(
                request,
                timeout=SIGNAL_TIMEOUT_S,
            ) as response:
                body = response.read().decode("utf-8-sig")

            parsed = json.loads(body)
            return latest_signal_record(parsed, itst_id)

        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                "HTTP %s: %s" % (error.code, detail[:160])
            )

        except (
            urllib.error.URLError,
            TimeoutError,
            OSError,
        ) as error:
            last_error = error

            if attempt < SIGNAL_RETRIES:
                time.sleep(float(attempt))

        except json.JSONDecodeError:
            raise ValueError("Signal response is not JSON")

    raise RuntimeError("Signal request failed: %s" % last_error)


def raw_signal_values(data, direction):
    opposite = OPPOSITE_DIRECTION[direction]

    values = {}

    for candidate in (direction, opposite):
        field = candidate + "PdsgRmdrCs"
        values[field] = find_value(data, field)

    return values



def collect_signal_records(data):
    records = []

    if isinstance(data, dict):
        if (
            data.get("itstId") not in (None, "")
            and data.get("trsmUtcTime") not in (None, "")
        ):
            records.append(data)

        for value in data.values():
            records.extend(collect_signal_records(value))

    elif isinstance(data, list):
        for value in data:
            records.extend(collect_signal_records(value))

    return records


def latest_signal_record(data, itst_id):
    records = [
        record
        for record in collect_signal_records(data)
        if str(record.get("itstId")) == str(itst_id)
    ]

    if not records:
        records = collect_signal_records(data)

    if not records:
        raise ValueError("No signal timing record found")

    def timestamp(record):
        try:
            return float(record.get("trsmUtcTime"))
        except (TypeError, ValueError):
            return -1.0

    return max(records, key=timestamp)


def all_valid_signal_values(data):
    valid = []

    for direction in ("nt", "ne", "et", "se", "st", "sw", "wt", "nw"):
        field = direction + "PdsgRmdrCs"
        raw = find_value(data, field)

        if raw in (None, ""):
            continue

        try:
            value = int(float(raw))
        except (TypeError, ValueError):
            continue

        if value in INVALID_SIGNAL_VALUES or value < 0:
            continue

        valid.append((value / 10.0, field))

    return valid


def signal_remaining_for_crosswalk(data, direction):
    pair = (direction, OPPOSITE_DIRECTION[direction])
    valid = []
    raw_values = {}

    for candidate in pair:
        field = candidate + "PdsgRmdrCs"
        raw = find_value(data, field)
        raw_values[field] = raw

        if raw in (None, ""):
            continue

        try:
            value = int(float(raw))
        except (TypeError, ValueError):
            continue

        if value in INVALID_SIGNAL_VALUES or value < 0:
            continue

        valid.append((value / 10.0, field))

    if not valid:
        raise ValueError(
            "No valid signal in crosswalk pair: " + str(raw_values)
        )

    # If both opposite fields are valid, use the shorter remaining time.
    # This is the safer decision for a crossing-assist device.
    return min(valid, key=lambda item: item[0]), raw_values


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
    print("Loading API key...", flush=True)
    api_key = read_api_key()

    print("Loading crosswalk data...", flush=True)
    crosswalks = load_crosswalks()
    print("Crosswalk records:", len(crosswalks), flush=True)

    print("Opening GPS...", flush=True)
    gps, gps_port = open_gps()
    print("GPS ready:", gps_port, GPS_BAUD, flush=True)

    last_map_position = None
    last_map_update = 0.0
    itst_id = None
    last_log_time = 0.0

    signal_data_cache = None
    signal_cache_time = 0.0
    signal_last_error = ""

    fix = None
    crosswalk = None
    required_time = None
    signal_s = None
    signal_field = ""
    signal_raw_values = {}
    decision = "NO SIGNAL DATA"

    try:
        while True:
            cycle_start = time.monotonic()
            error_text = ""
            signal_s = None
            signal_field = ""
            signal_raw_values = {}
            decision = "NO SIGNAL DATA"

            try:
                fix = read_gps_fix(gps)
                lat = fix["latitude"]
                lon = fix["longitude"]
                save_current_position(lat, lon)

                crosswalk = nearest_crosswalk(crosswalks, lat, lon)
                crossing_direction = crosswalk["crossing_direction"]

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
                    signal_data_cache = None
                    signal_cache_time = 0.0

                required_time = (
                    crosswalk["length_m"] / WALKING_SPEED_MPS
                    + SAFETY_MARGIN_S
                )

                if (
                    crosswalk["edge_distance_m"]
                    > MAX_CROSSWALK_DISTANCE_M
                ):
                    decision = "NO CROSSWALK"
                    error_text = "Crosswalk is farther than %.0f m" % (
                        MAX_CROSSWALK_DISTANCE_M
                    )

                elif (
                    crosswalk["axis_alignment_error_deg"]
                    > MAX_AXIS_ALIGNMENT_ERROR_DEG
                    and crosswalk["edge_distance_m"]
                    > CROSSING_READY_DISTANCE_M
                ):
                    decision = "ALIGNMENT CHECK"
                    error_text = (
                        "User-to-crosswalk direction does not align "
                        "with crosswalk axis"
                    )

                elif itst_id:
                    should_refresh_signal = (
                        signal_data_cache is None
                        or now - signal_cache_time >= SIGNAL_REFRESH_INTERVAL_S
                    )

                    if should_refresh_signal:
                        try:
                            signal_data_cache = request_signal_data(
                                api_key,
                                itst_id,
                            )
                            signal_cache_time = time.monotonic()
                            signal_last_error = ""

                        except Exception as signal_error:
                            signal_last_error = str(signal_error)

                    cache_age = (
                        time.monotonic() - signal_cache_time
                        if signal_data_cache is not None
                        else None
                    )

                    if (
                        signal_data_cache is not None
                        and cache_age is not None
                        and cache_age <= SIGNAL_CACHE_MAX_AGE_S
                    ):
                        try:
                            (
                                (signal_s, signal_field),
                                signal_raw_values,
                            ) = signal_remaining_for_crosswalk(
                                signal_data_cache,
                                crossing_direction,
                            )

                        except Exception as signal_parse_error:
                            valid_values = all_valid_signal_values(
                                signal_data_cache
                            )
                            error_text = str(signal_parse_error)

                            if valid_values:
                                error_text += "; other valid=" + str(
                                    valid_values
                                )

                            decision = "DIRECTION CHECK"
                            signal_s = None
                            signal_field = ""

                        if signal_s is not None:
                            if (
                                crosswalk["edge_distance_m"]
                                > CROSSING_READY_DISTANCE_M
                            ):
                                decision = "APPROACHING"

                            elif signal_s >= required_time:
                                decision = "CAN CROSS"

                            else:
                                decision = "WAIT"

                    elif signal_last_error:
                        error_text = signal_last_error
                        decision = "NO SIGNAL DATA"

            except Exception as error:
                error_text = str(error)
                decision = "NO SIGNAL DATA"

            clear_screen()
            print("SMART CROSSWALK LIVE V4")
            print("=======================")
            print("GPS port          :", gps_port)
            print("Screen refresh    : 1 second")
            print("Signal refresh    : %.0f seconds" % SIGNAL_REFRESH_INTERVAL_S)
            print("Time              :", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

            if fix is not None:
                print("Latitude          : %.8f" % fix["latitude"])
                print("Longitude         : %.8f" % fix["longitude"])
            else:
                print("Latitude          : -")
                print("Longitude         : -")

            if crosswalk is not None:
                print("Crosswalk edge    : %.1f m" % crosswalk["edge_distance_m"])
                print("Crosswalk center  : %.1f m" % crosswalk["center_distance_m"])
                print("Crosswalk len     : %.1f m" % crosswalk["length_m"])

                if crosswalk["width_m"] is not None:
                    print("Crosswalk width   : %.1f m" % crosswalk["width_m"])
                else:
                    print("Crosswalk width   : -")

                print(
                    "Crosswalk axis    : %.1f deg"
                    % crosswalk["axis_bearing_deg"]
                )
                print(
                    "Crossing direction:",
                    DIRECTION_NAMES.get(
                        crosswalk["crossing_direction"],
                        "-",
                    ),
                    "(" + crosswalk["crossing_direction"] + ")",
                )
                print(
                    "Axis alignment err: %.1f deg"
                    % crosswalk["axis_alignment_error_deg"]
                )
            else:
                print("Crosswalk edge    : -")
                print("Crosswalk center  : -")
                print("Crosswalk len     : -")
                print("Crosswalk width   : -")
                print("Crosswalk axis    : -")
                print("Crossing direction: -")
                print("Axis alignment err: -")

            print("Intersection ID   :", itst_id or "-")
            print("Walking speed     : %.2f m/s" % WALKING_SPEED_MPS)
            print("Safety margin     : %.1f s" % SAFETY_MARGIN_S)
            print(
                "Ready distance    : %.1f m"
                % CROSSING_READY_DISTANCE_M
            )

            if required_time is not None:
                print("Required time     : %.1f s" % required_time)
            else:
                print("Required time     : -")

            if signal_s is not None:
                print("Signal field      :", signal_field)
                print("Signal remains    : %.1f s" % signal_s)
            else:
                print("Signal field      : -")
                print("Signal remains    : -")

            print("Signal pair raw   :", signal_raw_values or "-")

            if signal_data_cache is not None:
                age = time.monotonic() - signal_cache_time
                print("Signal data age   : %.1f s" % age)
                print(
                    "Signal trsmUtc    :",
                    signal_data_cache.get("trsmUtcTime", "-"),
                )
            else:
                print("Signal data age   : -")
                print("Signal trsmUtc    : -")

            print()
            print("RESULT            :", decision)

            if error_text:
                print("STATUS            :", error_text)
            elif signal_last_error:
                print("STATUS            :", signal_last_error)
            else:
                print("STATUS            : OK")

            print()
            print("Press Ctrl+C to stop.")

            now_wall = time.monotonic()

            if (
                fix is not None
                and crosswalk is not None
                and now_wall - last_log_time >= 5.0
            ):
                append_log(
                    {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "latitude": "%.8f" % fix["latitude"],
                        "longitude": "%.8f" % fix["longitude"],
                        "itstId": itst_id or "",
                        "crossing_direction": crosswalk["crossing_direction"],
                        "crosswalk_edge_distance_m": round(
                            crosswalk["edge_distance_m"],
                            2,
                        ),
                        "crosswalk_center_distance_m": round(
                            crosswalk["center_distance_m"],
                            2,
                        ),
                        "crosswalk_length_m": round(
                            crosswalk["length_m"],
                            2,
                        ),
                        "crosswalk_axis_bearing_deg": round(
                            crosswalk["axis_bearing_deg"],
                            1,
                        ),
                        "walking_speed_mps": WALKING_SPEED_MPS,
                        "required_time_s": (
                            round(required_time, 1)
                            if required_time is not None
                            else ""
                        ),
                        "signal_field": signal_field,
                        "signal_remaining_s": (
                            round(signal_s, 1)
                            if signal_s is not None
                            else ""
                        ),
                        "decision": decision,
                        "status": error_text or signal_last_error or "OK",
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
