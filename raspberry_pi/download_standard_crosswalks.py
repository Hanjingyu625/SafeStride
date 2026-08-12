import json
import subprocess

API_URL = "https://api.data.go.kr/openapi/tn_pubr_public_crosswalk_api"
PAGE_SIZE = 1000
SEOUL = "\uc11c\uc6b8\ud2b9\ubcc4\uc2dc"


def get_items(data):
    body = data.get("response", {}).get("body", {})
    items = body.get("items", [])

    if isinstance(items, dict):
        items = items.get("item", [])

    if isinstance(items, dict):
        items = [items]

    if not isinstance(items, list):
        items = []

    total = int(body.get("totalCount", 0) or 0)

    return items, total


with open("public_api_key.txt") as file:
    api_key = file.read().strip()

all_rows = []
page = 1

while True:
    result = subprocess.run(
        [
            "curl",
            "-sS",
            "-G",
            API_URL,
            "--data-urlencode"
,
            "serviceKey=" + api_key,
            "--data",
            "pageNo=" + str(page),
            "--data",
            "numOfRows=" + str(PAGE_SIZE),
            "--data",
            "type=json",
            "--data-urlencode",
            "ctprvnNm=" + SEOUL
        ],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print("Curl error:", result.stderr)
        break

    try:
        data = json.loads(result.stdout)
    except json.JSONDecodeError:
        print("JSON error")
        print(result.stdout[:500])
        break

    items, total = get_items(data)

    print(
        "Page:",
        page,
        "Rows:",
        len(items),
        "Saved:",
        len(all_rows),
        "Total:",
        total
    )

    if not items:
        break

    all_rows.extend(items)

    if total and len(all_rows) >= total:
        break

    page += 1

with open("standard_crosswalks.json", "w") as file:
    json.dump(
        all_rows,
        file,
        ensure_ascii=False,
        indent=2
    )

print("Finished:", len(all_rows))
print("Saved: standard_crosswalks.json")

if all_rows:
    print("First row fields:")
    for key, value in all_rows[0].items():
        print(key, ":", value)
