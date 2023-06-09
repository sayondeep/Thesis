import csv

# Read the CSV file
csv_filename = "modified_file.csv"
output_filename = "vehicle_coordinates_with_probability.csv"

# Dictionary to store the count of requests for each file
file_requests = {}

# Variable to keep track of the total number of requests
total_requests = 0

# Process the CSV file and add probability column
with open(csv_filename, "r") as csv_file, open(output_filename, "w", newline="") as output_file:
    csv_reader = csv.DictReader(csv_file)
    fieldnames = csv_reader.fieldnames + ["Probability"]
    csv_writer = csv.DictWriter(output_file, fieldnames=fieldnames)
    csv_writer.writeheader()

    for row in csv_reader:
        tile_name = row["Tile Name"]

        # Increment the count for the file
        file_requests[tile_name] = file_requests.get(tile_name, 0) + 1

        # Increment the total number of requests
        total_requests += 1

        # Calculate the running probability for each file
        file_probabilities = {}
        for file_name, request_count in file_requests.items():
            probability = request_count / total_requests
            file_probabilities[file_name] = probability

        # Add the running probability to the current row and write to the output file
        row["Probability"] = file_probabilities[tile_name]
        csv_writer.writerow(row)
