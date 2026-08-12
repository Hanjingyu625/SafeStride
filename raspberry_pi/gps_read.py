PORT = "/dev/serial0"


def nmea_to_decimal(value, hemisphere):
    if not value:
        return None

    if hemisphere in ("N", "S"):
        degree_length = 2
    else:
        degree_length = 3

    degrees = float(value[:degree_length])
    minutes = float(value[degree_length:])

    coordinate = degrees + minutes / 60.0

    if hemisphere in ("S", "W"):
        coordinate = -coordinate

    return coordinate


print("GPS started")
print("Press Ctrl+C to stop")

try:
    with open(PORT, "rb", buffering=0) as gps:
        while True:
            raw_data = gps.readline()

            if not raw_data:
                continue

            line = raw_data.decode("ascii", errors="ignore").strip()

            if not line:
                continue

            if line.startswith(("$GNGGA", "$GPGGA")):
                parts = line.split(",")

                if len(parts) < 10:
                    continue

                latitude_raw = parts[2]
                latitude_direction = parts[3]
                longitude_raw = parts[4]
                longitude_direction = parts[5]
                fix_quality = parts[6]
                satellites = parts[7]
                altitude = parts[9]

                if fix_quality == "0" or not latitude_raw or not longitude_raw:
                    print("Searching for satellites...")
                    continue

                latitude = nmea_to_decimal(
                    latitude_raw,
                    latitude_direction
                )

                longitude = nmea_to_decimal(
                    longitude_raw,
                    longitude_direction
                )

                print("-------------------------")
                print(f"Latitude: {latitude:.7f}")
                print(f"Longitude: {longitude:.7f}")
                print(f"Satellites: {satellites}")
                print(f"Altitude: {altitude} m")

except KeyboardInterrupt:
    print("\nGPS stopped")

except PermissionError:
    print("Permission error")

except Exception as error:
    print("Error:", error)
