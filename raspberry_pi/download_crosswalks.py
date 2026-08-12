import json
import time
import urllib.request

SERVICE = "tbTraficCrsng"
PAGE_SIZE = 1000

with open("seoul_api_key.txt") as file:
    api_key = file.read().strip()

all_rows = []
start = 1

while True:
    end = start + PAGE_SIZE - 1

    url = (
        f"http://openapi.seoul.go.kr:8088/"
        f"{api_key}/json/{SERVICE}/{start}/{end}/"
    )

    print("Downloading:", start, "-", end)

    with urllib.request.urlopen(url, timeout=30) as response:
        data = json.load(response)

    body = data.get(SERVICE)

    if body is None:
        print("API error:", data)
        break

    rows = body.get("row", [])
    total = int(body.get("list_total_count", 0))

    all_rows.extend(rows)

    print("Saved:", len(all_rows), "/", total)

    if not rows or len(all_rows) >= total:
        break

    start = end + 1
    time.sleep(0.2)

with open("crosswalks.json", "w") as file:
    json.dump(all_rows, file, ensure_ascii=False, indent=2)

print("Finished")
print("Total:", len(all_rows))
print("Saved: crosswalks.json")

