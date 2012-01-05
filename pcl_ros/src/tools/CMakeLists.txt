# ---[ Tools
rosbuild_add_executable (pcd_to_pointcloud pcd_to_pointcloud.cpp)
target_link_libraries (pcd_to_pointcloud pcl_io)

rosbuild_add_executable (pointcloud_to_pcd pointcloud_to_pcd.cpp)
target_link_libraries (pointcloud_to_pcd pcl_io)

rosbuild_add_executable (bag_to_pcd bag_to_pcd.cpp)
target_link_libraries (bag_to_pcd pcl_io pcl_ros_tf)
rosbuild_link_boost (bag_to_pcd system filesystem)

rosbuild_add_executable (convert_pcd_to_image convert_pcd_to_image.cpp)
target_link_libraries (convert_pcd_to_image pcl_io pcl_common)

rosbuild_add_executable (convert_pointcloud_to_image convert_pointcloud_to_image.cpp)
target_link_libraries (convert_pointcloud_to_image pcl_io)

