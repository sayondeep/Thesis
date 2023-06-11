from pyntcloud import PyntCloud

def get_pcd_grid_size(file_path):
    # Read the PCD file
    point_cloud = PyntCloud.from_file(file_path)

    # Calculate the grid size
    print(point_cloud.xyz.max(axis=0))
    print(point_cloud.xyz.min(axis=0))
    grid_size = point_cloud.xyz.max(axis=0) - point_cloud.xyz.min(axis=0)

    return grid_size

# Specify the path to the PCD file

pcd_file = "/home/sayon/autoware_map/town01/pointcloud_map.pcd"

# Call the function to get the grid size
grid_size = get_pcd_grid_size(pcd_file)

# Print the grid size
print("Grid Size (X, Y, Z):", grid_size)






