from pyntcloud import PyntCloud

import yaml


def get_pcd_grid_size(file_path):
    # Read the PCD file
    point_cloud = PyntCloud.from_file(file_path)

    # Calculate the grid size
    # print(point_cloud.xyz.max(axis=0))
    # print(point_cloud.xyz.min(axis=0))
    # grid_size = point_cloud.xyz.max(axis=0) - point_cloud.xyz.min(axis=0)

    # return grid_size

    return point_cloud.xyz.min(axis=0),point_cloud.xyz.max(axis=0)

# Specify the path to the PCD file

pcd_file = "/home/sayon/autoware_map/town01/pointcloud_map.pcd"

# Call the function to get the grid size
mini,maxi = get_pcd_grid_size(pcd_file)

# Print the grid size
print(mini)
print(maxi)


import yaml

# Path to your YAML file
yaml_file_path = '/home/sayon/autoware_map/town01/splitted_64/64_metadata.yaml'

def convert_coordinates_to_indices(x, y, min_x, max_x, min_y, max_y, matrix_rows, matrix_columns):
    x_scaled = int((x - min_x) / (max_x - min_x) * (matrix_rows - 1))
    y_scaled = int((y - min_y) / (max_y - min_y) * (matrix_columns - 1))
    return x_scaled, y_scaled

with open(yaml_file_path, 'r') as file:
    yaml_data = yaml.safe_load(file)

x_resolution = yaml_data['x_resolution']
y_resolution = yaml_data['y_resolution']

min_x = mini[0]
max_x = maxi[0]
min_y = mini[1]
max_y = maxi[1]

# Get matrix size from the user
matrix_size = 64
matrix_rows = matrix_size
matrix_columns = matrix_size

matrix = [[None] * matrix_columns for _ in range(matrix_rows)]

# Iterate over the remaining coordinates
for file_name, coordinates in yaml_data.items():
    if file_name == 'x_resolution' or file_name == 'y_resolution':
        continue
    
    x, y = coordinates
    row, column = convert_coordinates_to_indices(x, y, min_x, max_x, min_y, max_y, matrix_rows, matrix_columns)
    matrix[row][column] = file_name

print(matrix)
