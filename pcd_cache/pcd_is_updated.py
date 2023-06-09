import csv
import numpy as np
from pyntcloud import PyntCloud

def get_min_max():
    cloud = PyntCloud.from_file("/home/sayon/autoware_map/town01/pointcloud_map.pcd")

    # Retrieve the point cloud data
    points = cloud.points
    # print(points)
    # print(type(points))

    # Find the minimum and maximum coordinates
    min_x, min_y, min_z = points.min(axis=0)[:3]
    max_x, max_y, max_z = points.max(axis=0)[:3]

    # Calculate the grid size
    grid_size_x = max_x - min_x
    grid_size_y = max_y - min_y
    grid_size_z = max_z - min_z

    # Determine the total grid size

    # print(max_x)
    # print(min_x)
    # # print(grid_size_x)
    # print(max_y)
    # print(min_y)
    # print(grid_size_y)
    # total_grid_size = grid_size_x * grid_size_y * grid_size_z

    # print("Total grid size:", total_grid_size)
    return min_x,max_x,min_y,max_y

def load_binary_matrix(filename):
    try:
        with open(filename, "r") as file:
            lines = file.readlines()
            
            # Get the dimensions of the matrix from the first line
            dimensions = lines[0].strip().split()
            matrix_size = int(dimensions[0])
            
            # Initialize an empty matrix
            matrix = np.zeros((matrix_size, matrix_size), dtype=np.uint8)
            
            # Fill the matrix with values from the text file
            for i, line in enumerate(lines[1:]):
                row_values = list(map(int, line.strip().split()))
                matrix[i] = row_values
            
            return matrix
    
    except FileNotFoundError:
        print("Matrix file not found.")
        return None

def check_location(matrix, x, y):
    if matrix is not None:
        matrix_size = matrix.shape[0]

        # Convert x and y to match the scaled matrix
        min_x, max_x, min_y, max_y = get_min_max()
        x_scaled = int((x - min_x) / (max_x - min_x) * (matrix_size - 1))
        y_scaled = int(abs(y - max_y) / (max_y - min_y) * (matrix_size - 1))

        if 0 <= x_scaled < matrix_size and 0 <= y_scaled < matrix_size:
            value = matrix[y_scaled, x_scaled]
            return value

        else:
            print("Invalid location coordinates.")
            return None

    else:
        return None


def add_is_update_column(csv_filename, matrix):
    try:
        updated_rows = []
        with open(csv_filename, "r") as csv_file:
            csv_reader = csv.DictReader(csv_file)

            if csv_reader.fieldnames != ["Time", "Vehicle ID", "X Coordinate", "Y Coordinate", "Tile Name", "Probability"]:
                raise ValueError("Invalid CSV file format.")

            fieldnames = csv_reader.fieldnames + ["Is_Update"]

            for row in csv_reader:
                x = float(row["X Coordinate"])
                y = float(row["Y Coordinate"])

                is_update = check_location(matrix, x, y)

                updated_rows.append(dict(row, Is_Update=is_update))

        with open("updated_data.csv", "w", newline="") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            csv_writer.writeheader()
            csv_writer.writerows(updated_rows)

        print("Is_Update column added to updated_data.csv")

    except FileNotFoundError:
        print("CSV file not found.")
    except ValueError as e:
        print("Error:", str(e))


# Load the matrix
matrix_filename = "modified_matrix.txt"
matrix = load_binary_matrix(matrix_filename)

# Update the CSV file with the Is_Update column
add_is_update_column("vehicle_coordinates_with_probability.csv", matrix)
