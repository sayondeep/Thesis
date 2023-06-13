import pyntcloud
import numpy as np
import open3d as o3d
import time

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
pcd_file = '/home/sayon/autoware_map/town01/splitted_4/4_-51_-64.pcd'

# Load the point cloud
point_cloud = pyntcloud.PyntCloud.from_file(pcd_file)

# Voxel size (adjust according to your needs)
voxel_size = 1  # Each voxel will be a cube with dimensions 0.1x0.1x0.1

# # Voxelize the point cloud
# voxel_grid, min_point = voxelize_point_cloud(point_cloud, voxel_size)

# # Create a voxel grid point cloud using open3d
# voxel_pc = o3d.geometry.PointCloud()
# voxel_pc.points = o3d.utility.Vector3dVector(min_point + voxel_size * np.argwhere(voxel_grid))

# print(np.asarray(voxel_pc.points))

# # Save the voxelized point cloud as a .pcd file
# o3d.io.write_point_cloud("voxelized_point_cloud.pcd", voxel_pc)

# # Print the voxel grid shape and the number of occupied voxels
# print("Voxel grid shape:", voxel_grid.shape)
# print("Number of occupied voxels:", np.sum(voxel_grid))



# Start the timer
start_time = time.time()

# Voxelize the point cloud
voxel_grid, min_point = voxelize_point_cloud(point_cloud, voxel_size)

# Calculate the elapsed time
elapsed_time = time.time() - start_time

# Print the elapsed time
print("Time taken to voxelize the point cloud:", elapsed_time, "seconds")
