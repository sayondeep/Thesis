import open3d as o3d
import numpy as np

# Path to the .pcd file
pcd_file = '/home/sayon/autoware_map/town01/pointcloud_map.pcd'

# Load the point cloud
point_cloud = o3d.io.read_point_cloud(pcd_file)

# Voxel size (adjust according to your needs)
voxel_size = 0.1  # Each voxel will be a cube with dimensions 0.1x0.1x0.1

# Calculate the grid size based on the voxel size
min_point = np.min(np.asarray(point_cloud.points), axis=0)
max_point = np.max(np.asarray(point_cloud.points), axis=0)
grid_size = np.ceil((max_point - min_point) / voxel_size).astype(int)

# Calculate the voxel indices for each point
voxel_indices = np.floor((np.asarray(point_cloud.points) - min_point) / voxel_size).astype(int)

# Initialize the voxel grid
voxel_grid = np.zeros(grid_size, dtype=bool)

# Set the occupied voxels
voxel_grid[voxel_indices[:, 0], voxel_indices[:, 1], voxel_indices[:, 2]] = True

# Print the voxel grid shape and the number of occupied and unoccupied voxels
total_voxels = np.prod(voxel_grid.shape)
occupied_voxels = np.sum(voxel_grid)
unoccupied_voxels = total_voxels - occupied_voxels
print("Voxel grid shape:", voxel_grid.shape)
print("Number of occupied voxels:", occupied_voxels)
print("Number of unoccupied voxels:", unoccupied_voxels)
