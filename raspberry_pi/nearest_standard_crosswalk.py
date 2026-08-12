#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent

POS_FILE = BASE / "current_position.csv"
DATA_FILE = BASE / "standard_crosswalks.json"
OUT_FILE = BASE / "nearest_standard_crosswalk.json"


def num(value):
    try:
        if value is None or str(value).strip() == "":
            return None

        return float(
            str(value).replace(",", "").strip()
        )

    except ValueError:
        return None


def haversine(lat1, lon1, lat2, lon2):
    radius = 6371000.0

    p1 = math.radians(lat1)
    p2 = math.radians(lat2)

    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = (
        math.sin(dp / 2) ** 2
        + math.cos(p1)
        * math.cos(p2)
        * math.sin(dl / 2) ** 2
    )

    return (
        2
        * radius
        * math.asin(math.sqrt(a))
    )


def main():
    if not POS_FILE.exists():
        raise FileNotFoundError(
            "current_position.csv not found"
        )

    if not DATA_FILE.exists():
        raise FileNotFoundError(
            "standard_crosswalks.json not found"
        )

    with POS_FILE.open(
        newline="",
        encoding="utf-8-sig"
    ) as file:

        position = next(
            csv.DictReader(file),
            None
        )

    if position is None:
        raise ValueError(
            "current_position.csv is empty"
        )

    user_lat = num(
        position.get("latitude")
    )

    user_lon = num(
        position.get("longitude")
    )

    if user_lat is None or user_lon is None:
        raise ValueError(
            "latitude or longitude is missing"
        )

    with DATA_FILE.open(
        encoding="utf-8-sig"
    ) as file:

        rows = json.load(file)

    best = None

    for row in rows:
        lat = num(
            row.get("latitude")
        )

        lon = num(
            row.get("longitude")
        )

        length = num(
            row.get("crswlkLt") or row.get("et")
        )

        width = num(
            row.get("crswlkBt") or row.get("bt")
        )

        if lat is None or lon is None:
            continue

        if length is None or length <= 0:
            continue

        dist = haversine(
            user_lat,
            user_lon,
            lat,
            lon
        )

        if (
            best is None
            or dist < best["distance_m"]
        ):
            best = {
                "latitude": lat,
                "longitude": lon,
                "distance_m": dist,
                "length_m": length,
                "width_m": (
                    0.0
                    if width is None
                    else width
                ),
                "source_record": row
            }

    if best is None:
        raise ValueError(
            "No crosswalk with valid "
            "coordinates and crswlkLt"
        )

    best["distance_m"] = round(
        best["distance_m"],
        2
    )

    best["length_m"] = round(
        best["length_m"],
        2
    )

    best["width_m"] = round(
        best["width_m"],
        2
    )

    with OUT_FILE.open(
        "w",
        encoding="utf-8"
    ) as file:

        json.dump(
            best,
            file,
            ensure_ascii=False,
            indent=2
        )

    print(
        "Nearest standard crosswalk"
    )

    print(
        "Distance:",
        best["distance_m"],
        "m"
    )

    print(
        "Length:",
        best["length_m"],
        "m"
    )

    print(
        "Width:",
        best["width_m"],
        "m"
    )

    print(
        "Saved:",
        OUT_FILE.name
    )


if __name__ == "__main__":
    main()

