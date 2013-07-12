cmake_minimum_required(VERSION 2.8)
project(pcl_ros)

# Deal with catkin
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(catkin REQUIRED dynamic_reconfigure genmsg nodelet nodelet_topic_tools pcl_conversions pcl_msgs roscpp sensor_msgs std_msgs tf rosbag rosconsole roslib pluginlib)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

# deal with ROS
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${Eigen_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/Feature.cfg
                                     cfg/Filter.cfg
                                     cfg/EuclideanClusterExtraction.cfg
                                     cfg/ExtractIndices.cfg 
                                     cfg/ExtractPolygonalPrismData.cfg
                                     cfg/MLS.cfg
                                     cfg/SACSegmentation.cfg
                                     cfg/SACSegmentationFromNormals.cfg
                                     cfg/SegmentDifferences.cfg 
                                     cfg/StatisticalOutlierRemoval.cfg
                                     cfg/VoxelGrid.cfg

)
include_directories(include cfg/cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pcl_ros_tf pcl_ros_io pcl_ros_filters
  CATKIN_DEPENDS roscpp sensor_msgs tf sensor_msgs std_msgs
  DEPENDS Eigen PCL 
)

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

class_loader_hide_library_symbols(pcl_ros_io)

# ---[ PCL ROS - Features
add_library (pcl_ros_features
              src/pcl_ros/features/feature.cpp
              # Compilation is much faster if we include all the following CPP files in feature.cpp
              src/pcl_ros/features/boundary.cpp
              src/pcl_ros/features/fpfh.cpp
              src/pcl_ros/features/fpfh_omp.cpp
              src/pcl_ros/features/moment_invariants.cpp
              src/pcl_ros/features/normal_3d.cpp
              src/pcl_ros/features/normal_3d_omp.cpp
              src/pcl_ros/features/pfh.cpp
              src/pcl_ros/features/principal_curvatures.cpp
              src/pcl_ros/features/vfh.cpp
              )
#rosbuild_add_compile_flags (pcl_ros_features ${SSE_FLAGS})
target_link_libraries (pcl_ros_features ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

class_loader_hide_library_symbols(pcl_ros_features)


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

class_loader_hide_library_symbols(pcl_ros_filters)

# ---[ Point Cloud Library - Segmentation
add_library (pcl_ros_segmentation
              src/pcl_ros/segmentation/segmentation.cpp
              src/pcl_ros/segmentation/segment_differences.cpp
              src/pcl_ros/segmentation/extract_clusters.cpp
              src/pcl_ros/segmentation/extract_polygonal_prism_data.cpp
              src/pcl_ros/segmentation/sac_segmentation.cpp
              )
#rosbuild_add_compile_flags (pcl_ros_segmentation ${SSE_FLAGS})
target_link_libraries (pcl_ros_segmentation pcl_segmentation pcl_ros_tf ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

class_loader_hide_library_symbols(pcl_ros_segmentation)

# ---[ Point Cloud Library - Surface
add_library (pcl_ros_surface
              src/pcl_ros/surface/surface.cpp
              # Compilation is much faster if we include all the following CPP files in surface.cpp
              src/pcl_ros/surface/convex_hull.cpp
              src/pcl_ros/surface/moving_least_squares.cpp
              )
#rosbuild_add_compile_flags (pcl_ros_surface ${SSE_FLAGS})
target_link_libraries (pcl_ros_surface ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

class_loader_hide_library_symbols(pcl_ros_surface)

############ TOOLS

add_executable (pcd_to_pointcloud tools/pcd_to_pointcloud.cpp)
target_link_libraries (pcd_to_pointcloud pcl_io ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable (pointcloud_to_pcd tools/pointcloud_to_pcd.cpp)
target_link_libraries (pointcloud_to_pcd pcl_io ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable (bag_to_pcd tools/bag_to_pcd.cpp)
target_link_libraries (bag_to_pcd pcl_io pcl_ros_tf ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable (convert_pcd_to_image tools/convert_pcd_to_image.cpp)
target_link_libraries (convert_pcd_to_image pcl_io pcl_common ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable (convert_pointcloud_to_image tools/convert_pointcloud_to_image.cpp)
target_link_libraries (convert_pointcloud_to_image pcl_io ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS pcl_ros_tf pcl_ros_io pcl_ros_features pcl_ros_filters pcl_ros_surface pcl_ros_segmentation pcd_to_pointcloud pointcloud_to_pcd bag_to_pcd convert_pcd_to_image convert_pointcloud_to_image
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# these target locations are deprecated and will be dropped in Hydro!
#install(TARGETS pcd_to_pointcloud pointcloud_to_pcd bag_to_pcd convert_pcd_to_image convert_pointcloud_to_image
#  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


