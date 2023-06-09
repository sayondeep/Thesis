import pyntcloud

def extract_vehicle_pose_from_pcd(pcd_file_path):
    cloud = pyntcloud.PyntCloud.from_file(pcd_file_path)
    x = cloud.points['x'].mean()
    y = cloud.points['y'].mean()
    z = cloud.points['z'].mean()
    # roll = cloud.points['roll'].mean()
    # pitch = cloud.points['pitch'].mean()
    # yaw = cloud.points['yaw'].mean()
    #from metadata file
    roll = 0
    pitch = 0
    yaw = 0
    
    vehicle_pose = {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
    
    return vehicle_pose

# Example usage
pcd_file = '/home/sayon/autoware_map/town01/pointcloud_map.pcd'
vehicle_pose = extract_vehicle_pose_from_pcd(pcd_file)
print(vehicle_pose)
