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
from collections import deque
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
    "/dev/ttyS0",
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

# Automatic crossing state-machine thresholds.
APPROACH_DISTANCE_M = 50.0
LOCK_CROSSWALK_DISTANCE_M = 18.0
CURB_ZONE_M = 7.0
CURB_RELEASE_DISTANCE_M = 12.0
ENTRY_START_PROGRESS_M = 1.2
ENTRY_START_MIN_GAIN_M = 1.0
ENTRY_START_WINDOW_S = 4.0
ENTRY_MIN_SPEED_MPS = 0.15
EXIT_CLEARANCE_M = 1.5
EXIT_HOLD_S = 2.0
COMPLETE_HOLD_S = 4.0
CROSSING_TIMEOUT_S = 180.0
REACTION_TIME_S = 2.0
ENTRY_SAFETY_MARGIN_S = 5.0
CROSSING_TIME_MARGIN_S = 2.0
DEFAULT_SAFE_SPEED_MPS = 0.50
MIN_ESTIMATE_SPEED_MPS = 0.15
MAX_ASSIST_SPEED_MPS = 0.85
PROFILE_FILE = BASE_DIR / "user_speed_profile.json"
CONTROLLER_LOG_FILE = BASE_DIR / "crosswalk_controller_log.csv"

ARDUINO_PORT_CANDIDATES = (
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB1",
    "/dev/ttyUSB2",
    "/dev/ttyUSB0",
)
ARDUINO_BAUD = 115200
ARDUINO_COMMAND_PERIOD_S = 0.20
ARDUINO_RECONNECT_S = 3.0
ARDUINO_STATUS_TIMEOUT_S = 1.0

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

    signal_bearing = (crossing_bearing + 90.0) % 360.0

    output["target_bearing_deg"] = target_bearing
    output["crossing_bearing_deg"] = crossing_bearing
    output["crossing_direction"] = bearing_to_direction(crossing_bearing)
    output["signal_bearing_deg"] = signal_bearing
    output["signal_direction"] = bearing_to_direction(signal_bearing)
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



def clamp(value, low, high):
    return max(low, min(high, value))


def percentile(values, fraction):
    if not values:
        return None

    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]

    position = clamp(fraction, 0.0, 1.0) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))

    if lower == upper:
        return ordered[lower]

    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


class UserSpeedProfile:
    """Conservative online walking-speed profile.

    The controller uses the lower 20th percentile rather than the average so
    entry decisions remain conservative for an older user whose speed varies.
    """

    def __init__(self, path=PROFILE_FILE):
        self.path = Path(path)
        self.samples = deque(maxlen=300)
        self.last_save = 0.0
        self._load()

    def _load(self):
        if not self.path.exists():
            return

        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            for value in data.get("recent_speed_samples_mps", []):
                speed = number(value)
                if speed is not None and 0.12 <= speed <= 1.8:
                    self.samples.append(speed)
        except Exception:
            return

    def add(self, speed_mps, allow_update=True):
        speed = number(speed_mps)
        if not allow_update or speed is None:
            return

        if 0.12 <= speed <= 1.8:
            self.samples.append(speed)

        now = time.monotonic()
        if now - self.last_save >= 30.0:
            self.save()
            self.last_save = now

    def safe_speed(self):
        value = percentile(list(self.samples), 0.20)
        if value is None:
            return DEFAULT_SAFE_SPEED_MPS
        return clamp(value, 0.30, 1.00)

    def average_speed(self):
        if not self.samples:
            return DEFAULT_SAFE_SPEED_MPS
        return sum(self.samples) / len(self.samples)

    def save(self):
        payload = {
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "sample_count": len(self.samples),
            "safe_speed_mps": round(self.safe_speed(), 3),
            "average_speed_mps": round(self.average_speed(), 3),
            "recent_speed_samples_mps": [
                round(value, 3) for value in list(self.samples)[-120:]
            ],
        }
        temp = self.path.with_suffix(".tmp")
        temp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        temp.replace(self.path)


class ArduinoBridge:
    """Optional Pi-Arduino serial bridge.

    Pi to Arduino:
      CMD,seq,mode,target_speed,max_pwm,entry_allowed,alert,valid_ms,crc

    Arduino to Pi:
      STAT,seq,speed,distance,handle_l,handle_r,pitch,roll,tof,
      local_state,fault_flags,crc
    """

    def __init__(self, excluded_port=None):
        self.excluded_port = excluded_port
        self.device = None
        self.port = None
        self.last_connect_attempt = 0.0
        self.last_command_time = 0.0
        self.last_status_time = 0.0
        self.sequence = 0
        self.rx_buffer = ""
        self.status = {
            "speed_mps": None,
            "distance_m": None,
            "handle_left": None,
            "handle_right": None,
            "pitch_deg": None,
            "roll_deg": None,
            "tof_mm": None,
            "local_state": "OFFLINE",
            "fault_flags": "0",
        }

    @staticmethod
    def checksum(payload):
        value = 0
        for byte in payload.encode("ascii", errors="ignore"):
            value ^= byte
        return "%02X" % value

    @classmethod
    def encode(cls, fields):
        payload = ",".join(str(value) for value in fields)
        return payload + "," + cls.checksum(payload) + "\n"

    @classmethod
    def verify(cls, line):
        parts = line.strip().split(",")
        if len(parts) < 3:
            return None

        payload = ",".join(parts[:-1])
        if cls.checksum(payload).upper() != parts[-1].upper():
            return None
        return parts[:-1]

    def connected(self):
        return self.device is not None and self.device.is_open

    def status_fresh(self):
        return (
            self.connected()
            and time.monotonic() - self.last_status_time
            <= ARDUINO_STATUS_TIMEOUT_S
        )

    def _close(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
        self.device = None
        self.port = None
        self.status["local_state"] = "OFFLINE"

    def connect_if_needed(self):
        if self.connected():
            return

        now = time.monotonic()
        if now - self.last_connect_attempt < ARDUINO_RECONNECT_S:
            return
        self.last_connect_attempt = now

        for port in ARDUINO_PORT_CANDIDATES:
            if port == self.excluded_port or not os.path.exists(port):
                continue

            try:
                device = serial.Serial(
                    port=port,
                    baudrate=ARDUINO_BAUD,
                    timeout=0,
                    write_timeout=0.2,
                )
                time.sleep(0.25)
                device.reset_input_buffer()
                self.device = device
                self.port = port
                self.status["local_state"] = "CONNECTED"
                return
            except Exception:
                continue

    def poll(self):
        self.connect_if_needed()
        if not self.connected():
            return self.status

        try:
            waiting = self.device.in_waiting
            if waiting:
                data = self.device.read(waiting).decode(
                    "ascii",
                    errors="ignore",
                )
                self.rx_buffer += data

            while "\n" in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                fields = self.verify(line)
                if not fields or fields[0] != "STAT" or len(fields) < 11:
                    continue

                self.status = {
                    "sequence": fields[1],
                    "speed_mps": number(fields[2]),
                    "distance_m": number(fields[3]),
                    "handle_left": fields[4] == "1",
                    "handle_right": fields[5] == "1",
                    "pitch_deg": number(fields[6]),
                    "roll_deg": number(fields[7]),
                    "tof_mm": number(fields[8]),
                    "local_state": fields[9],
                    "fault_flags": fields[10],
                }
                self.last_status_time = time.monotonic()

        except Exception:
            self._close()

        return self.status

    def send_command(self, command):
        self.connect_if_needed()
        if not self.connected():
            return False

        now = time.monotonic()
        if now - self.last_command_time < ARDUINO_COMMAND_PERIOD_S:
            return True

        self.sequence = (self.sequence + 1) % 100000
        fields = (
            "CMD",
            self.sequence,
            command["mode"],
            "%.2f" % command["target_speed_mps"],
            int(command["max_pwm_pct"]),
            1 if command["entry_allowed"] else 0,
            int(command["alert_code"]),
            int(command["valid_ms"]),
        )

        try:
            self.device.write(self.encode(fields).encode("ascii"))
            self.last_command_time = now
            return True
        except Exception:
            self._close()
            return False

    def close(self):
        self._close()


def crosswalk_axis_position(item, lat, lon, crossing_bearing_deg):
    east, north = local_offset_m(
        item["latitude"],
        item["longitude"],
        lat,
        lon,
    )

    bearing = math.radians(crossing_bearing_deg)
    axis_east = math.sin(bearing)
    axis_north = math.cos(bearing)
    side_east = math.sin(bearing + math.pi / 2.0)
    side_north = math.cos(bearing + math.pi / 2.0)

    along = east * axis_east + north * axis_north
    across = east * side_east + north * side_north
    progress = along + item["length_m"] / 2.0
    remaining = max(item["length_m"] - progress, 0.0)

    return {
        "along_m": along,
        "across_m": across,
        "progress_m": progress,
        "remaining_m": remaining,
        "lateral_error_m": max(
            abs(across) - max(item.get("width_m") or 0.0, 0.0) / 2.0,
            0.0,
        ),
    }


def evaluate_locked_crosswalk(item, lat, lon):
    output = dict(item)
    output["center_distance_m"] = haversine_m(
        lat,
        lon,
        item["latitude"],
        item["longitude"],
    )
    output["edge_distance_m"] = crosswalk_edge_distance_m(item, lat, lon)
    output.update(
        crosswalk_axis_position(
            item,
            lat,
            lon,
            item["crossing_bearing_deg"],
        )
    )
    return output


class CrossingStateMachine:
    STATES = (
        "IDLE",
        "APPROACHING",
        "WAIT_AT_CURB",
        "ENTRY_ALLOWED",
        "CROSSING",
        "CROSSING_URGENT",
        "EXITING",
    )

    def __init__(self):
        self.state = "IDLE"
        self.state_since = time.monotonic()
        self.locked_crosswalk = None
        self.locked_itst_id = None
        self.progress_history = deque(maxlen=12)
        self.exit_seen_since = None
        self.crossing_started_at = None
        self.arm_encoder_origin = None
        self.crossing_encoder_origin = None
        self.reason = "Waiting for a crosswalk"

    def set_state(self, new_state, reason):
        if new_state not in self.STATES:
            raise ValueError("Invalid crossing state: " + str(new_state))

        if new_state != self.state:
            self.state = new_state
            self.state_since = time.monotonic()

            if new_state in ("WAIT_AT_CURB", "ENTRY_ALLOWED"):
                self.arm_encoder_origin = None

            if new_state in ("CROSSING", "CROSSING_URGENT"):
                if self.crossing_started_at is None:
                    self.crossing_started_at = self.state_since

            if new_state == "EXITING":
                self.exit_seen_since = self.state_since

        self.reason = reason

    def lock(self, crosswalk, itst_id):
        if self.locked_crosswalk is not None:
            return

        self.locked_crosswalk = dict(crosswalk)
        self.locked_itst_id = itst_id
        self.progress_history.clear()

    def reset(self, reason="Reset"):
        self.state = "IDLE"
        self.state_since = time.monotonic()
        self.locked_crosswalk = None
        self.locked_itst_id = None
        self.progress_history.clear()
        self.exit_seen_since = None
        self.crossing_started_at = None
        self.arm_encoder_origin = None
        self.crossing_encoder_origin = None
        self.reason = reason

    def current_crosswalk(self, candidate, lat, lon):
        if self.locked_crosswalk is not None:
            return evaluate_locked_crosswalk(
                self.locked_crosswalk,
                lat,
                lon,
            )
        return candidate

    def _record_progress(self, progress_m):
        now = time.monotonic()
        self.progress_history.append((now, progress_m))

        while (
            self.progress_history
            and now - self.progress_history[0][0] > ENTRY_START_WINDOW_S
        ):
            self.progress_history.popleft()

    def _automatic_start_detected(
        self,
        progress_m,
        speed_mps,
        encoder_distance_m,
    ):
        self._record_progress(progress_m)

        if speed_mps is None or speed_mps < ENTRY_MIN_SPEED_MPS:
            return False

        if self.arm_encoder_origin is None and encoder_distance_m is not None:
            self.arm_encoder_origin = encoder_distance_m

        gps_started = False
        if progress_m >= ENTRY_START_PROGRESS_M and len(self.progress_history) >= 2:
            gain = progress_m - min(
                value for _, value in self.progress_history
            )
            gps_started = gain >= ENTRY_START_MIN_GAIN_M

        encoder_started = False
        if (
            encoder_distance_m is not None
            and self.arm_encoder_origin is not None
        ):
            encoder_gain = encoder_distance_m - self.arm_encoder_origin
            encoder_started = encoder_gain >= ENTRY_START_MIN_GAIN_M

        return gps_started or encoder_started

    def update(
        self,
        candidate,
        itst_id,
        lat,
        lon,
        signal_s,
        signal_valid,
        safe_speed_mps,
        measured_speed_mps,
        encoder_distance_m=None,
    ):
        now = time.monotonic()
        active = self.current_crosswalk(candidate, lat, lon)

        if active is None:
            self.reset("No crosswalk candidate")
            return None, None, None

        required_entry_time = (
            active["length_m"] / max(safe_speed_mps, MIN_ESTIMATE_SPEED_MPS)
            + REACTION_TIME_S
            + ENTRY_SAFETY_MARGIN_S
        )

        if self.state == "IDLE":
            if active["edge_distance_m"] <= APPROACH_DISTANCE_M:
                self.set_state("APPROACHING", "Crosswalk detected ahead")
            else:
                self.reason = "Crosswalk is outside approach range"

        if self.state == "APPROACHING":
            if active["edge_distance_m"] > APPROACH_DISTANCE_M + 10.0:
                self.reset("Moved away from crosswalk")
                return active, required_entry_time, None

            if (
                self.locked_crosswalk is None
                and active["edge_distance_m"] <= LOCK_CROSSWALK_DISTANCE_M
            ):
                self.lock(active, itst_id)
                active = self.current_crosswalk(candidate, lat, lon)

            if active["edge_distance_m"] <= CURB_ZONE_M:
                if signal_valid and signal_s >= required_entry_time:
                    self.set_state(
                        "ENTRY_ALLOWED",
                        "Enough signal time to enter",
                    )
                else:
                    self.set_state(
                        "WAIT_AT_CURB",
                        "Wait for a safer signal window",
                    )

        elif self.state == "WAIT_AT_CURB":
            progress = active.get("progress_m")

            if active["edge_distance_m"] > CURB_RELEASE_DISTANCE_M:
                self.set_state("APPROACHING", "User moved away from curb")
            elif (
                progress is not None
                and self._automatic_start_detected(
                    progress,
                    measured_speed_mps,
                    encoder_distance_m,
                )
            ):
                self.crossing_encoder_origin = self.arm_encoder_origin
                self.set_state(
                    "CROSSING_URGENT",
                    "Entry detected despite WAIT; continue across, do not stop",
                )
            elif signal_valid and signal_s >= required_entry_time:
                self.progress_history.clear()
                self.set_state(
                    "ENTRY_ALLOWED",
                    "Signal time became sufficient",
                )

        elif self.state == "ENTRY_ALLOWED":
            progress = active.get("progress_m")

            if active["edge_distance_m"] > CURB_RELEASE_DISTANCE_M:
                self.set_state("APPROACHING", "User moved away before entry")
            elif not signal_valid or signal_s < required_entry_time:
                self.set_state(
                    "WAIT_AT_CURB",
                    "Signal window closed before entry",
                )
            elif progress is not None and self._automatic_start_detected(
                progress,
                measured_speed_mps,
                encoder_distance_m,
            ):
                self.crossing_encoder_origin = self.arm_encoder_origin
                self.set_state(
                    "CROSSING",
                    "Automatic entry detected from position and motion",
                )

        elif self.state in ("CROSSING", "CROSSING_URGENT"):
            progress = active.get("progress_m")

            if (
                encoder_distance_m is not None
                and self.crossing_encoder_origin is not None
            ):
                encoder_progress = max(
                    encoder_distance_m - self.crossing_encoder_origin,
                    0.0,
                )
                if progress is None:
                    progress = encoder_progress
                else:
                    progress = max(progress, encoder_progress)
                active["progress_m"] = progress
                active["remaining_m"] = max(
                    active["length_m"] - progress,
                    0.0,
                )

            remaining = active.get("remaining_m")

            if progress is not None:
                self._record_progress(progress)

            estimate_speed = measured_speed_mps
            if estimate_speed is None or estimate_speed < MIN_ESTIMATE_SPEED_MPS:
                estimate_speed = max(
                    safe_speed_mps * 0.75,
                    MIN_ESTIMATE_SPEED_MPS,
                )

            eta_s = (
                remaining / estimate_speed
                if remaining is not None
                else None
            )

            if (
                progress is not None
                and progress >= active["length_m"] + EXIT_CLEARANCE_M
            ):
                if self.exit_seen_since is None:
                    self.exit_seen_since = now
                elif now - self.exit_seen_since >= EXIT_HOLD_S:
                    self.set_state("EXITING", "Far curb reached")
            else:
                self.exit_seen_since = None

            if self.state in ("CROSSING", "CROSSING_URGENT"):
                urgent = (
                    not signal_valid
                    or signal_s is None
                    or eta_s is None
                    or signal_s < eta_s + CROSSING_TIME_MARGIN_S
                )

                if urgent:
                    self.set_state(
                        "CROSSING_URGENT",
                        "Continue crossing; remaining signal is tight",
                    )
                else:
                    self.set_state(
                        "CROSSING",
                        "Crossing progress is within the signal window",
                    )

                if (
                    self.crossing_started_at is not None
                    and now - self.crossing_started_at > CROSSING_TIMEOUT_S
                ):
                    self.set_state(
                        "CROSSING_URGENT",
                        "Crossing timeout; continue assistance and alert",
                    )

            return active, required_entry_time, eta_s

        elif self.state == "EXITING":
            if now - self.state_since >= COMPLETE_HOLD_S:
                self.reset("Crossing completed")

        return active, required_entry_time, None

    def command(self, safe_speed_mps, measured_speed_mps, arduino_status):
        local_state = str(arduino_status.get("local_state", "OFFLINE"))
        handles_known = (
            arduino_status.get("handle_left") is not None
            and arduino_status.get("handle_right") is not None
        )
        handles_held = (
            arduino_status.get("handle_left") is True
            or arduino_status.get("handle_right") is True
        )

        # The Arduino still has final authority. This Pi-side command merely
        # agrees with a reported physical hazard and never overrides it.
        if local_state not in ("OK", "CONNECTED", "OFFLINE"):
            return {
                "mode": "HARD_STOP",
                "target_speed_mps": 0.0,
                "max_pwm_pct": 0,
                "entry_allowed": False,
                "alert_code": 9,
                "valid_ms": 500,
            }

        if handles_known and not handles_held:
            return {
                "mode": "HARD_STOP",
                "target_speed_mps": 0.0,
                "max_pwm_pct": 0,
                "entry_allowed": False,
                "alert_code": 9,
                "valid_ms": 500,
            }

        if self.state == "IDLE":
            mode, speed, pwm, allowed, alert = (
                "NORMAL_ASSIST",
                safe_speed_mps,
                50,
                False,
                0,
            )
        elif self.state == "APPROACHING":
            mode, speed, pwm, allowed, alert = (
                "SPEED_LIMIT",
                min(safe_speed_mps, 0.50),
                45,
                False,
                1,
            )
        elif self.state == "WAIT_AT_CURB":
            mode, speed, pwm, allowed, alert = (
                "SOFT_STOP",
                0.0,
                0,
                False,
                2,
            )
        elif self.state == "ENTRY_ALLOWED":
            mode, speed, pwm, allowed, alert = (
                "ENTRY_ALLOWED",
                safe_speed_mps,
                60,
                True,
                3,
            )
        elif self.state == "CROSSING":
            mode, speed, pwm, allowed, alert = (
                "CROSSING_ASSIST",
                max(safe_speed_mps, measured_speed_mps or 0.0),
                70,
                True,
                4,
            )
        elif self.state == "CROSSING_URGENT":
            base_speed = max(
                safe_speed_mps + 0.10,
                measured_speed_mps or 0.0,
            )
            mode, speed, pwm, allowed, alert = (
                "CROSSING_URGENT",
                min(base_speed, MAX_ASSIST_SPEED_MPS),
                80,
                True,
                5,
            )
        else:
            mode, speed, pwm, allowed, alert = (
                "NORMAL_ASSIST",
                min(safe_speed_mps, 0.50),
                50,
                True,
                6,
            )

        return {
            "mode": mode,
            "target_speed_mps": clamp(speed, 0.0, MAX_ASSIST_SPEED_MPS),
            "max_pwm_pct": pwm,
            "entry_allowed": allowed,
            "alert_code": alert,
            "valid_ms": 500,
        }


def append_controller_log(row):
    exists = (
        CONTROLLER_LOG_FILE.exists()
        and CONTROLLER_LOG_FILE.stat().st_size > 0
    )

    with CONTROLLER_LOG_FILE.open(
        "a",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(file, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)

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

    profile = UserSpeedProfile()
    controller = CrossingStateMachine()
    arduino = ArduinoBridge(excluded_port=gps_port)

    last_map_position = None
    last_map_update = 0.0
    itst_id = None
    last_log_time = 0.0
    last_fix = None
    filtered_speed_samples = deque(maxlen=5)

    signal_data_cache = None
    signal_cache_time = 0.0
    signal_last_error = ""

    fix = None
    crosswalk = None
    required_entry_time = None
    crossing_eta_s = None
    signal_s = None
    signal_field = ""
    signal_raw_values = {}
    error_text = ""

    try:
        while True:
            cycle_start = time.monotonic()
            signal_s = None
            signal_field = ""
            signal_raw_values = {}
            error_text = ""
            crossing_eta_s = None

            arduino_status = arduino.poll()

            try:
                fix = read_gps_fix(gps)
                lat = fix["latitude"]
                lon = fix["longitude"]
                save_current_position(lat, lon)

                gps_speed = fix.get("gps_speed_mps")
                if gps_speed is None and last_fix is not None:
                    dt = time.monotonic() - last_fix["monotonic"]
                    if dt > 0.2:
                        gps_speed = haversine_m(
                            last_fix["latitude"],
                            last_fix["longitude"],
                            lat,
                            lon,
                        ) / dt

                if gps_speed is not None and 0.0 <= gps_speed <= 3.0:
                    filtered_speed_samples.append(gps_speed)

                filtered_gps_speed = (
                    sum(filtered_speed_samples) / len(filtered_speed_samples)
                    if filtered_speed_samples
                    else None
                )
                last_fix = {
                    "latitude": lat,
                    "longitude": lon,
                    "monotonic": time.monotonic(),
                }

                measured_speed = arduino_status.get("speed_mps")
                speed_source = "ENCODER"
                if measured_speed is None or measured_speed < 0:
                    measured_speed = filtered_gps_speed
                    speed_source = "GPS"

                profile.add(
                    measured_speed,
                    allow_update=controller.state
                    not in ("WAIT_AT_CURB", "CROSSING_URGENT"),
                )
                safe_speed = profile.safe_speed()

                candidate = nearest_crosswalk(crosswalks, lat, lon)

                now = time.monotonic()
                map_distance = None
                if last_map_position is not None:
                    map_distance = haversine_m(
                        last_map_position[0],
                        last_map_position[1],
                        lat,
                        lon,
                    )

                map_locked = controller.locked_itst_id is not None
                need_map_refresh = (
                    not map_locked
                    and (
                        itst_id is None
                        or now - last_map_update >= MAP_REFRESH_INTERVAL_S
                        or (
                            map_distance is not None
                            and map_distance >= MAP_REFRESH_DISTANCE_M
                        )
                    )
                )

                if need_map_refresh:
                    itst_id = refresh_nearest_crossroad()
                    last_map_position = (lat, lon)
                    last_map_update = time.monotonic()
                    signal_data_cache = None
                    signal_cache_time = 0.0

                active_itst_id = controller.locked_itst_id or itst_id
                active_for_signal = controller.current_crosswalk(
                    candidate,
                    lat,
                    lon,
                )
                signal_direction = active_for_signal["signal_direction"]

                if active_itst_id:
                    should_refresh_signal = (
                        signal_data_cache is None
                        or now - signal_cache_time
                        >= SIGNAL_REFRESH_INTERVAL_S
                    )

                    if should_refresh_signal:
                        try:
                            signal_data_cache = request_signal_data(
                                api_key,
                                active_itst_id,
                            )
                            signal_cache_time = time.monotonic()
                            signal_last_error = ""
                        except Exception as signal_error:
                            signal_last_error = str(signal_error)

                signal_valid = False
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
                            signal_direction,
                        )
                        signal_valid = True
                    except Exception as signal_error:
                        error_text = str(signal_error)
                        valid_values = all_valid_signal_values(
                            signal_data_cache
                        )
                        if valid_values:
                            error_text += "; other valid=" + str(
                                valid_values
                            )
                elif signal_last_error:
                    error_text = signal_last_error

                crosswalk, required_entry_time, crossing_eta_s = (
                    controller.update(
                        candidate=candidate,
                        itst_id=active_itst_id,
                        lat=lat,
                        lon=lon,
                        signal_s=signal_s,
                        signal_valid=signal_valid,
                        safe_speed_mps=safe_speed,
                        measured_speed_mps=measured_speed,
                        encoder_distance_m=arduino_status.get("distance_m"),
                    )
                )

                command = controller.command(
                    safe_speed,
                    measured_speed,
                    arduino_status,
                )
                arduino.send_command(command)

            except Exception as error:
                error_text = str(error)
                safe_speed = profile.safe_speed()
                measured_speed = None
                speed_source = "-"
                command = {
                    "mode": "SOFT_STOP",
                    "target_speed_mps": 0.0,
                    "max_pwm_pct": 0,
                    "entry_allowed": False,
                    "alert_code": 8,
                    "valid_ms": 500,
                }
                arduino.send_command(command)

            clear_screen()
            print("SMART CROSSWALK CONTROLLER V6")
            print("=============================")
            print("Time              :", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            print("GPS port          :", gps_port)
            print("Arduino port      :", arduino.port or "OFFLINE")
            print("Arduino state     :", arduino_status.get("local_state", "OFFLINE"))
            print("Arduino status age:", end=" ")
            if arduino.last_status_time > 0:
                print("%.1f s" % (time.monotonic() - arduino.last_status_time))
            else:
                print("-")

            if fix is not None:
                print("Latitude          : %.8f" % fix["latitude"])
                print("Longitude         : %.8f" % fix["longitude"])
            else:
                print("Latitude          : -")
                print("Longitude         : -")

            print("Measured speed    :", end=" ")
            if measured_speed is not None:
                print("%.2f m/s (%s)" % (measured_speed, speed_source))
            else:
                print("-")
            print("User safe speed   : %.2f m/s" % safe_speed)
            print("Profile samples   :", len(profile.samples))

            if crosswalk is not None:
                print("Crosswalk edge    : %.1f m" % crosswalk["edge_distance_m"])
                print("Crosswalk len     : %.1f m" % crosswalk["length_m"])
                print("Crosswalk width   : %.1f m" % (crosswalk.get("width_m") or 0.0))
                print("Crossing direction:", DIRECTION_NAMES.get(crosswalk["crossing_direction"], "-"))
                print("Signal direction  :", DIRECTION_NAMES.get(crosswalk["signal_direction"], "-"))
                if "progress_m" in crosswalk:
                    print("Crossing progress : %.1f m" % crosswalk["progress_m"])
                    print("Remaining distance: %.1f m" % crosswalk["remaining_m"])
                    print("Lateral error     : %.1f m" % crosswalk["lateral_error_m"])
                else:
                    print("Crossing progress : -")
                    print("Remaining distance: -")
                    print("Lateral error     : -")
            else:
                print("Crosswalk edge    : -")
                print("Crosswalk len     : -")
                print("Crosswalk width   : -")
                print("Crossing direction: -")
                print("Signal direction  : -")
                print("Crossing progress : -")
                print("Remaining distance: -")
                print("Lateral error     : -")

            print("Intersection ID   :", controller.locked_itst_id or itst_id or "-")
            print("Signal field      :", signal_field or "-")
            print("Signal remains    :", "%.1f s" % signal_s if signal_s is not None else "-")
            print("Required entry    :", "%.1f s" % required_entry_time if required_entry_time is not None else "-")
            print("Crossing ETA      :", "%.1f s" % crossing_eta_s if crossing_eta_s is not None else "-")
            print()
            print("STATE             :", controller.state)
            print("STATE REASON      :", controller.reason)
            print("PI COMMAND        :", command["mode"])
            print("Target speed      : %.2f m/s" % command["target_speed_mps"])
            print("Max PWM           : %d %%" % command["max_pwm_pct"])
            print("Entry allowed     :", command["entry_allowed"])
            print("Alert code        :", command["alert_code"])

            if error_text:
                print("STATUS            :", error_text)
            elif not arduino.connected():
                print("STATUS            : Pi-only test; Arduino not connected")
            else:
                print("STATUS            : OK")

            print()
            print("Automatic start: no button is required.")
            print("Press Ctrl+C to stop.")

            now_wall = time.monotonic()
            if (
                fix is not None
                and crosswalk is not None
                and now_wall - last_log_time >= 2.0
            ):
                append_controller_log(
                    {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "state": controller.state,
                        "state_reason": controller.reason,
                        "latitude": "%.8f" % fix["latitude"],
                        "longitude": "%.8f" % fix["longitude"],
                        "itstId": controller.locked_itst_id or itst_id or "",
                        "crosswalk_index": crosswalk.get("index", ""),
                        "edge_distance_m": round(crosswalk["edge_distance_m"], 2),
                        "progress_m": round(crosswalk.get("progress_m", 0.0), 2),
                        "remaining_m": round(crosswalk.get("remaining_m", 0.0), 2),
                        "safe_speed_mps": round(safe_speed, 3),
                        "measured_speed_mps": (
                            round(measured_speed, 3)
                            if measured_speed is not None
                            else ""
                        ),
                        "signal_remaining_s": (
                            round(signal_s, 1)
                            if signal_s is not None
                            else ""
                        ),
                        "required_entry_s": (
                            round(required_entry_time, 1)
                            if required_entry_time is not None
                            else ""
                        ),
                        "crossing_eta_s": (
                            round(crossing_eta_s, 1)
                            if crossing_eta_s is not None
                            else ""
                        ),
                        "pi_command": command["mode"],
                        "target_speed_mps": command["target_speed_mps"],
                        "max_pwm_pct": command["max_pwm_pct"],
                        "arduino_state": arduino_status.get("local_state", "OFFLINE"),
                        "fault_flags": arduino_status.get("fault_flags", "0"),
                        "status": error_text or "OK",
                    }
                )
                last_log_time = now_wall

            elapsed = time.monotonic() - cycle_start
            time.sleep(max(0.0, UPDATE_INTERVAL_S - elapsed))

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            profile.save()
        except Exception:
            pass
        arduino.close()
        gps.close()


if __name__ == "__main__":
    main()
