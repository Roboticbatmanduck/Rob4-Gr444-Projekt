import csv
import glob
import os

# Path to your CSV folder
folder_path = "theList/"  # adjust if needed
csv_files = glob.glob(os.path.join(folder_path, "*.csv"))  # gets all CSVs in folder
output_file = "compiled_angles.csv"

with open(output_file, "w", newline="") as out_csv:
    writer = csv.writer(out_csv)
    
    for file_index, csv_file in enumerate(csv_files):
        filename = os.path.basename(csv_file)
        # Write a row indicating which file the angles are from
        writer.writerow([f"Angles from file: {filename}"])
        writer.writerow(["angle_deg"])  # column header
        
        with open(csv_file, "r") as in_csv:
            reader = csv.DictReader(in_csv)
            for row in reader:
                writer.writerow([row["angle_deg"]])
        
        # Add an empty row after each file for readability
        writer.writerow([])

print(f"All angles compiled into '{output_file}' successfully!")