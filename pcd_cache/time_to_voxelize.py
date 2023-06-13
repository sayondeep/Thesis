import pyntcloud
import numpy as np
import time
import pandas as pd

def voxelize_point_cloud(point_cloud, voxel_size):
    # Calculate the grid size based on the voxel size
    min_point = np.min(point_cloud.xyz, axis=0)
    max_point = np.max(point_cloud.xyz, axis=0)
    grid_size = np.ceil((max_point - min_point + voxel_size) / voxel_size).astype(int)  # Adjusted grid size calculation

    # Calculate the voxel indices for each point
    voxel_indices = np.floor((point_cloud.xyz - min_point) / voxel_size).astype(int)

    # Initialize the voxel grid
    voxel_grid = np.zeros(grid_size, dtype=bool)

    # Set the occupied voxels
    voxel_grid[voxel_indices[:, 0], voxel_indices[:, 1], voxel_indices[:, 2]] = True

    return voxel_grid, min_point

# Path to the .pcd file
# pcd_file = '/home/sayon/autoware_map/town01/pointcloud_map.pcd'

# Load the point cloud
# point_cloud = pyntcloud.PyntCloud.from_file(pcd_file)

# Voxel size (adjust according to your needs)
voxel_size = 0.5  # Each voxel will be a cube with dimensions 0.1x0.1x0.1

# Load the CSV file
data = pd.read_csv('validity_with_size.csv')

# Create an empty list to store the voxelization times
times = []

# Iterate over each row in the CSV file
for index, row in data.iterrows():
    # Extract the tile name from the row
    tile_name = row['Tile Name']

    if(not pd.isna(tile_name)):

        # Start the timer
        start_time = time.time()

        print(f"{tile_name} to be voxelized.")
        # Voxelize the tile name
        # Modify this part according to your requirements
        point_cloud = pyntcloud.PyntCloud.from_file(tile_name)
        # ...
        voxel_grid, min_point = voxelize_point_cloud(point_cloud, voxel_size)

        # Calculate the elapsed time
        elapsed_time = time.time() - start_time

        # print(f"{tile_name} voxelized.")
    
    else:

        elapsed_time = 0

    # Append the voxelization time to the list
    times.append(elapsed_time)

# Add the voxelization times to the DataFrame as a new column
data['time_to_voxelize'] = times

# Save the updated DataFrame to a new CSV file
data.to_csv('time_to_voxelize.csv', index=False)
