from pyntcloud import PyntCloud

# Load the .pcd file
cloud = PyntCloud.from_file("/home/sayon/autoware_map/town01/pointcloud_map.pcd")

# Retrieve the point cloud data
points = cloud.points
print(points)
print(type(points))

# Find the minimum and maximum coordinates
min_x, min_y, min_z = points.min(axis=0)[:3]
max_x, max_y, max_z = points.max(axis=0)[:3]

# Calculate the grid size
grid_size_x = max_x - min_x
grid_size_y = max_y - min_y
grid_size_z = max_z - min_z

# Determine the total grid size

print(max_x)
print(min_x)
print(grid_size_x)
print(max_y)
print(min_y)
print(grid_size_y)
total_grid_size = grid_size_x * grid_size_y * grid_size_z

print("Total grid size:", total_grid_size)

