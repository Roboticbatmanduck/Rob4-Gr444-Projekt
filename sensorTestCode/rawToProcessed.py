import csv
import json
import math

# Input and output CSV files
input_file = "raw_serial_log_3m_4.csv"
output_file = "processed_data4_3_4m.csv"

def cm_to_m(cm):
    """Convert centimeters to meters rounded to 2 decimals."""
    return round(cm / 100, 2)

def calculate_angle(x, y):
    """Calculate angle in degrees from x, y using atan2."""
    return round(math.degrees(math.atan2(x, y)),5)

with open(input_file, "r", newline="") as f_in, open(output_file, "w", newline="") as f_out:
    reader = csv.reader(f_in)
    writer = csv.writer(f_out)
    
    # Write header including angle
    writer.writerow(["timestamp", "distance_raw_m", "x_m", "y_m", "angle_deg"])
    
    for row in reader:
        try:
            timestamp, raw_data = row
            # Clean the string (remove b'' and JS006E prefix)
            json_str = raw_data.strip().lstrip("b'JS006E").rstrip("'\\r\\n")
            
            # Convert string to JSON
            data_json = json.loads(json_str)
            twr = data_json["TWR"]
            
            # Extract distance and positions
            distance = twr["D"]
            x_m = cm_to_m(twr["Xcm"])
            y_m = cm_to_m(twr["Ycm"])
            
            # Calculate angle
            angle = calculate_angle(x_m, y_m)
            
            writer.writerow([timestamp, distance, x_m, y_m, angle])
        
        except Exception as e:
            print(f"Skipping row due to error: {e}")

print(f"Processed CSV saved to {output_file}")