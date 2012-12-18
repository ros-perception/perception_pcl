cmake_minimum_required(VERSION 2.8)
project(pcl_ros)

# Deal with catkin
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(catkin REQUIRED dynamic_reconfigure genmsg roscpp sensor_msgs std_msgs tf rosbag rosconsole roslib)
find_package(Eigen)
find_package(PCL)

# deal with ROS
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${Eigen_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/Filter.cfg
                                     cfg/ExtractIndices.cfg 
                                     cfg/StatisticalOutlierRemoval.cfg
                                     cfg/VoxelGrid.cfg

)
include_directories(include cfg/cpp)

catkin_package()

# ---[ Point Cloud Library - Transforms
add_library (pcl_ros_tf SHARED src/transforms.cpp)
target_link_libraries(pcl_ros_tf ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pcl_ros_tf ros_gencpp pcl_ros_copy)

############ NODELETS

# ---[ Point Cloud Library - IO
add_library (pcl_ros_io
              src/pcl_ros/io/io.cpp
              src/pcl_ros/io/concatenate_fields.cpp
              src/pcl_ros/io/concatenate_data.cpp
              src/pcl_ros/io/pcd_io.cpp
              src/pcl_ros/io/bag_io.cpp
             )
#rosbuild_add_compile_flags (pcl_ros_io ${SSE_FLAGS})
target_link_libraries (pcl_ros_io pcl_ros_tf ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# ---[ PCL ROS - Filters
add_library (pcl_ros_filters
	      src/pcl_ros/filters/filter.cpp
	      src/pcl_ros/filters/passthrough.cpp
	      src/pcl_ros/filters/project_inliers.cpp
	      src/pcl_ros/filters/extract_indices.cpp
	      src/pcl_ros/filters/radius_outlier_removal.cpp
	      src/pcl_ros/filters/statistical_outlier_removal.cpp
	      src/pcl_ros/filters/voxel_grid.cpp
	     )
#add_compile_flags (pcl_ros_filters ${SSE_FLAGS})
target_link_libraries (pcl_ros_filters pcl_ros_tf ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

############ TOOLS

add_executable (pcd_to_pointcloud tools/pcd_to_pointcloud.cpp)
target_link_libraries (pcd_to_pointcloud pcl_io ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable (pointcloud_to_pcd tools/pointcloud_to_pcd.cpp)
target_link_libraries (pointcloud_to_pcd pcl_io ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable ( bag_to_pcd tools/bag_to_pcd.cpp)
target_link_libraries (bag_to_pcd pcl_io pcl_ros_tf ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable (convert_pcd_to_image tools/convert_pcd_to_image.cpp)
target_link_libraries (convert_pcd_to_image pcl_io pcl_common ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable (convert_pointcloud_to_image tools/convert_pointcloud_to_image.cpp)
target_link_libraries (convert_pointcloud_to_image pcl_io ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS pcl_ros_tf pcl_ros_io pcl_ros_filters
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pcd_to_pointcloud pointcloud_to_pcd bag_to_pcd convert_pcd_to_image convert_pointcloud_to_image
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


