import csv
import os
import numpy as np
from scipy.stats import expon

def update_validity(csv_filename):
    map_data = {}

    updated_rows = []

    with open(csv_filename, "r") as csv_file:
        csv_reader = csv.DictReader(csv_file)

        if csv_reader.fieldnames != ["Time", "Vehicle ID", "X Coordinate", "Y Coordinate", "Tile Name", "Probability", "Is_Update"]:
            raise ValueError("Invalid CSV file format.")

        fieldnames = csv_reader.fieldnames + ["Valid_From", "Valid_Till", "Size"]

        for row in csv_reader:
            tile_name = row["Tile Name"]
            time = int(float(row["Time"]))
            is_update = int(row["Is_Update"])

            if tile_name not in map_data:
                valid_from = time
                if is_update == 1:
                    valid_till = time + np.random.exponential(scale=10)
                else:
                    valid_till = int("501") #max.simulation time+1

                map_data[tile_name] = {"valid_from": valid_from, "valid_till": valid_till}

            else:
                if map_data[tile_name]["valid_till"] < time:
                    valid_from = time
                    valid_till = time + np.random.exponential(scale=10)
                    map_data[tile_name]["valid_from"] = valid_from
                    map_data[tile_name]["valid_till"] = valid_till

                else:
                    valid_from = map_data[tile_name]["valid_from"]
                    valid_till = map_data[tile_name]["valid_till"]

            # Update the row with the "Valid_From" and "Valid_Till" columns
            row.update({"Valid_From": valid_from, "Valid_Till": valid_till})

            # Get the file size of the tile name file in MB
            if(tile_name):
                file_size_mb = os.path.getsize(tile_name) / (1024 * 1024)
            else:
                file_size_mb = 0
            row.update({"Size": file_size_mb})

            updated_rows.append(row)

    new_csv_filename = "validity_with_size.csv"

    with open(new_csv_filename, "w", newline="") as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
        csv_writer.writerows(updated_rows)

    print(f"Validity updated in {new_csv_filename}")

# Example usage
update_validity("updated_data.csv")
