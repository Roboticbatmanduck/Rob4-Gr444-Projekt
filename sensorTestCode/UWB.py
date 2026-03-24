import serial
import json
import csv
from datetime import datetime

# Open USB serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Open CSV file
with open("UWB_output_angle.csv", "a", newline="") as csvfile:
    writer = csv.writer(csvfile)

    # Write header (only once if file is empty)
    csvfile.seek(0, 2)  # move to end
    if csvfile.tell() == 0:
        writer.writerow([
            "timestamp",
            "distance_raw_m",
            "angle_raw",
            "x_m",
            "y_m"
        ])

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        if not line.startswith("JS006B"):
            continue

        try:
            json_start = line.find('{')
            if json_start == -1:
                continue

            data = json.loads(line[json_start:])
            twr = data.get("TWR", {})

            p_value = twr.get("P", None)
            d_value = twr.get("D", None)
            x_cm = twr.get("Xcm", None)
            y_cm = twr.get("Ycm", None)

            if None in (p_value, d_value, x_cm, y_cm):
                continue

            # Convert units
            distance_m = d_value / 100.0
            x_m = x_cm / 100.0
            y_m = y_cm / 100.0

            # Timestamp
            timestamp = datetime.now().isoformat()

            # Print
            print(
                f"Distance={distance_m:.2f} m | Angle={p_value:.2f} | X={x_m:.2f} m | Y={y_m:.2f} m"
            )

            # Write to CSV
            writer.writerow([
                timestamp,
                distance_m,
                p_value,
                x_m,
                y_m
            ])
            csvfile.flush()

        except json.JSONDecodeError:
            print("JSON error:", line)

        except Exception as e:
            print("Error:", e)