PORT = "/dev/serial0"
SAMPLE_COUNT = 10


def nmea_to_decimal(value, hemisphere):
    if hemisphere in ("N", "S"):
        degree_length = 2
    else:
        degree_length = 3

    degrees = float(value[:degree_length])
    minutes = float(value[degree_length:])
    result = degrees + minutes / 60.0

    if hemisphere in ("S", "W"):
        result = -result

    return result


latitudes = []
longitudes = []

print("Collecting GPS samples...")

try:
    with open(PORT, "rb", buffering=0) as gps:
        while len(latitudes) < SAMPLE_COUNT:
            line = gps.readline().decode("ascii", errors="ignore").strip()

            if not line.startswith(("$GNGGA", "$GPGGA")):
                continue

            parts = line.split(",")

            if len(parts) < 10:
                continue

            latitude_raw = parts[2]
            latitude_direction = parts[3]
            longitude_raw = parts[4]
            longitude_direction = parts[5]
            fix_quality = parts[6]

            if fix_quality == "0":
                print("Waiting for satellite fix...")
                continue

            if not latitude_raw or not longitude_raw:
                continue

            latitude = nmea_to_decimal(
                latitude_raw,
                latitude_direction
            )

            longitude = nmea_to_decimal(
                longitude_raw,
                longitude_direction
            )

            latitudes.append(latitude)
            longitudes.append(longitude)

            print(
                f"Sample {len(latitudes)}/{SAMPLE_COUNT}: "
                f"{latitude:.7f}, {longitude:.7f}"
            )

    average_latitude = sum(latitudes) / len(latitudes)
    average_longitude = sum(longitudes) / len(longitudes)

    print("-------------------------")
    print(f"Average Latitude: {average_latitude:.7f}")
    print(f"Average Longitude: {average_longitude:.7f}")

    with open("current_position.csv", "w") as file:
        file.write("latitude,longitude\n")
        file.write(
            f"{average_latitude:.7f},"
            f"{average_longitude:.7f}\n"
        )

    print("Saved: current_position.csv")

except KeyboardInterrupt:
    print("\nStopped")
