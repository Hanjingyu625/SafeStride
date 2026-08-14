import serial
import time
import csv
import json
import subprocess
import sys
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path


CURRENT_POSITION_FILE = Path("current_position.csv")
NEAREST_CROSSROAD_FILE = Path("nearest_crossroad.json")
NEAREST_CROSSWALK_FILE = Path("nearest_standard_crosswalk.json")
TDATA_KEY_FILE = Path("api_key.txt")
LOG_FILE = Path("crosswalk_log.csv")

TIMING_URL = (
    "https://t-data.seoul.go.kr/apig/apiman-gateway/"
    "tapi/v2xSignalPhaseTimingInformation/1.0"
)

DIRECTIONS = [
    "nt",
    "ne",
    "et",
    "se",
    "st",
    "sw",
    "wt",
    "nw"
]

INVALID_SIGNAL_VALUES = {
    36001,
    36000,
    -1
}


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


def run_script(filename):
    print()
    print("Running:", filename)

    result = subprocess.run(
        [sys.executable, filename],
        text=True
    )

    if result.returncode != 0:
        raise RuntimeError(filename + " failed")


def load_current_position():
    if not CURRENT_POSITION_FILE.exists():
        raise FileNotFoundError(
            "current_position.csv not found"
        )

    with CURRENT_POSITION_FILE.open(
        newline="",
        encoding="utf-8-sig"
    ) as file:
        row = next(csv.DictReader(file))

    latitude = float(row["latitude"])
    longitude = float(row["longitude"])

    return latitude, longitude


def load_itst_id():
    if not NEAREST_CROSSROAD_FILE.exists():
        raise FileNotFoundError(
            "nearest_crossroad.json not found"
        )

    with NEAREST_CROSSROAD_FILE.open(
        encoding="utf-8"
    ) as file:
        data = json.load(file)

    itst_id = find_value(data, "itstId")

    if itst_id is None:
        raise ValueError(
            "itstId not found"
        )

    return str(itst_id)


def load_crosswalk():
    if not NEAREST_CROSSWALK_FILE.exists():
        raise FileNotFoundError(
            "nearest_standard_crosswalk.json not found"
        )

    with NEAREST_CROSSWALK_FILE.open(
        encoding="utf-8"
    ) as file:
        data = json.load(file)

    length = float(data["length_m"])
    width = float(data["width_m"])
    distance = float(data["distance_m"])

    return data, length, width, distance

def send_to_arduino(command):
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)
        ser.write((command + '\n').encode())
        ser.close()
        print("Arduino command:", command)
    except Exception as e:
        print("Arduino send failed:", e)

def get_signal_remaining(
    api_key,
    itst_id,
    direction
):
    field_name = direction + "PdsgRmdrCs"

    query = urllib.parse.urlencode({
        "apikey": api_key,
        "itstId": itst_id
    })

    url = TIMING_URL + "?" + query

    request = urllib.request.Request(
        url,
        headers={
            "User-Agent": "smart-crosswalk"
        }
    )

    with urllib.request.urlopen(
        request,
        timeout=10
    ) as response:
        text = response.read().decode("utf-8")
        data = json.loads(text)

    raw_value = find_value(
        data,
        field_name
    )

    if raw_value is None:
        raise ValueError(
            field_name + " not found"
        )

    raw_value = int(float(raw_value))

    if raw_value in INVALID_SIGNAL_VALUES:
        raise ValueError(
            "Invalid signal value: "
            + str(raw_value)
        )

    if raw_value < 0:
        raise ValueError(
            "Negative signal value"
        )

    remaining_seconds = raw_value / 10.0

    return remaining_seconds, field_name


def input_number(prompt, minimum):
    while True:
        text = input(prompt).strip()

        try:
            value = float(text)

            if value < minimum:
                raise ValueError

            return value

        except ValueError:
            print("Enter a valid number")
def append_log(
    latitude,
    longitude,
    itst_id,
    direction,
    crosswalk_distance,
    crosswalk_length,
    crosswalk_width,
    walking_speed,
    signal_remaining,
    required_time,
    decision,
    reason
):
    fields = [
        "timestamp",
        "latitude",
        "longitude",
        "itstId",
        "direction",
        "crosswalk_distance_m",
        "crosswalk_length_m",
        "crosswalk_width_m",
        "walking_speed_mps",
        "signal_remaining_s",
        "required_time_s",
        "decision",
        "reason"
    ]

    exists = LOG_FILE.exists()

    row = {
        "timestamp": datetime.now().isoformat(
            timespec="seconds"
        ),
        "latitude": latitude,
        "longitude": longitude,
        "itstId": itst_id,
        "direction": direction,
        "crosswalk_distance_m": round(
            crosswalk_distance,
            2
        ),
        "crosswalk_length_m": crosswalk_length,
        "crosswalk_width_m": crosswalk_width,
        "walking_speed_mps": walking_speed,
        "signal_remaining_s": signal_remaining,
        "required_time_s": round(
            required_time,
            2
        ),
        "decision": decision,
        "reason": reason
    }

    with LOG_FILE.open(
        "a",
        newline="",
        encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fields
        )

        if not exists:
            writer.writeheader()

        writer.writerow(row)
def main():
    print(
        "=== SMART CROSSWALK MAIN ==="
    )

    latitude, longitude = (
        load_current_position()
    )

    print(
        "Current GPS:",
        latitude,
        longitude
    )

    run_script("nearest_map.py")

    itst_id = load_itst_id()

    print(
        "Nearest itstId:",
        itst_id
    )

    run_script(
        "nearest_standard_crosswalk.py"
    )

    (
        crosswalk,
        crosswalk_length,
        crosswalk_width,
        crosswalk_distance
    ) = load_crosswalk()

    print()
    print(
        "Crosswalk distance:",
        round(crosswalk_distance, 1),
        "m"
    )

    print(
        "Crosswalk length:",
        crosswalk_length,
        "m"
    )

    print(
        "Crosswalk width:",
        crosswalk_width,
        "m"
    )

    while True:
        direction = input(
            "Direction "
            "[nt/ne/et/se/st/sw/wt/nw]: "
        ).strip().lower()

        if direction in DIRECTIONS:
            break

        print("Invalid direction")

    walking_speed = input_number(
        "Walking speed m/s "
        "(example 0.6): ",
        0.05
    )

    safety_margin = input_number(
        "Safety margin seconds "
        "(example 3): ",
        0.0
    )

    required_time = (
        crosswalk_length
        / walking_speed
        + safety_margin
    )

    decision = "WAIT"
    reason = ""
    signal_remaining = ""

    try:
        if not TDATA_KEY_FILE.exists():
            raise FileNotFoundError(
                "api_key.txt not found"
            )

        api_key = TDATA_KEY_FILE.read_text(
            encoding="utf-8"
        ).strip()

        if not api_key:
            raise ValueError(
                "api_key.txt is empty"
            )

        (
            signal_remaining,
            signal_field
        ) = get_signal_remaining(
            api_key,
            itst_id,
            direction
        )

        print()
        print(
            "Signal field:",
            signal_field
        )

        print(
            "Signal remaining:",
            round(signal_remaining, 1),
            "seconds"
        )

        print(
            "Required time:",
            round(required_time, 1),
            "seconds"
        )

        if crosswalk_distance > 60:
            reason = (
                "Crosswalk is too far"
            )

        elif signal_remaining > required_time:
            decision = "CAN CROSS"
            send_to_arduino("CROSS")
            reason = (
                "Enough signal time"
            )

        else:
            reason = (
                "Not enough signal time"
            )

    except Exception as error:
        decision = "WAIT"
        send_to_arduino("WAIT")
        reason = (
            "Signal or API error: "
            + str(error)
        )

    print()
    print(
        "DECISION:",
        decision
    )

    print(
        "REASON:",
        reason
    )

    append_log(
        latitude,
        longitude,
        itst_id,
        direction,
        crosswalk_distance,
        crosswalk_length,
        crosswalk_width,
        walking_speed,
        signal_remaining,
        required_time,
        decision,
        reason
    )

    print()
    print(
        "Saved:",
        LOG_FILE
    )
if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print()
        print("Stopped")

    except Exception as error:
        print()
        print(
            "ERROR:",
            error
        )
