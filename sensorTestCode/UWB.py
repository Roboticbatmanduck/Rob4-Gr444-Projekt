import serial
import csv
from datetime import datetime

# Open USB serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=3)

# Open CSV file
with open("raw_serial_log_3m_4.csv", "a", newline="") as csvfile:
    writer = csv.writer(csvfile)

    # Write header if file is empty
    csvfile.seek(0, 2)
    if csvfile.tell() == 0:
        writer.writerow(["timestamp", "raw_data"])

    while True:
        line = ser.readline()

        if not line:
            continue

        # Timestamp
        timestamp = datetime.now().isoformat()

        # Convert bytes to string safely for CSV storage
        raw_str = repr(line)  # keeps exact byte representation

        # Print to terminal
        print(raw_str)

        # Write to CSV
        writer.writerow([timestamp, raw_str])
        csvfile.flush()