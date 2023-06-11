from pypcd import pypcd
# also can read from file handles.
pc = pypcd.PointCloud.from_path('/home/sayon/autoware_map/town02/pointcloud_map.pcd')
# pc.pc_data has the data as a structured array
# pc.fields, pc.count, etc have the metadata

# center the x field
# pc.pc_data['x'] -= pc.pc_data['x'].mean()

# save as binary compressed
pc.save_pcd('bar.pcd', compression='binary')