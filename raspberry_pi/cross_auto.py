import json
import subprocess
import urllib.parse

URL = "http://t-data.seoul.go.kr/apig/apiman-gateway/tapi/v2xSignalPhaseTimingInformation/1.0"

key = input("Timing API key: ").strip()
direction = input("Direction (nt/et/st/wt): ").strip().lower()
width = float(input("Crosswalk width (m): "))
speed = float(input("Walking speed (m/s): "))

with open("nearest_crossroad.json") as file:
    crossroad = json.load(file)

itst_id = str(crossroad["itstId"])

params = {
    "apiKey": key,
    "type": "json",
    "pageNo": 1,
    "numOfRows": 100,
    "itstId": itst_id
}

url = URL + "?" + urllib.parse.urlencode(params)

result = subprocess.run(
    ["curl", "-sS", "-L", "--http1.1", url],
    capture_output=True,
    text=True
)

data = json.loads(result.stdout)

rows = []


def find_rows(value):
    if isinstance(value, dict):
        if str(value.get("itstId", "")) == itst_id:
            rows.append(value)

        for item in value.values():
            find_rows(item)

    elif isinstance(value, list):
        for item in value:
            find_rows(item)


find_rows(data)

if not rows:
    print("No signal timing data")
    raise SystemExit

latest = rows[-1]

field = direction + "PdsgRmdrCs"
raw_time = latest.get(field)

if raw_time in (None, ""):
    print("No remaining time for this direction")
    raise SystemExit

raw_time = float(raw_time)

if raw_time >= 36000:
    print("Invalid remaining time")
    raise SystemExit

remaining_time = raw_time / 10.0
safety_margin = 5.0
crossing_time = width / speed
required_time = crossing_time + safety_margin

print()
print("Crossroad:", crossroad.get("name", ""))
print("itstId:", itst_id)
print("Direction:", direction)
print("Remaining time:", round(remaining_time, 1), "sec")
print("Crossing time:", round(crossing_time, 1), "sec")
print("Required time:", round(required_time, 1), "sec")

if remaining_time >= required_time:
    print("Result: CAN CROSS")
else:
    print("Result: WAIT")
