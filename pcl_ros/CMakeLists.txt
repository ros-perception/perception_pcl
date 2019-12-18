cmake_minimum_required(VERSION 3.5)
project(pcl_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # For debian: https://github.com/ros-perception/perception_pcl/issues/139
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
# find_package(rosbag2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)


ament_export_dependencies(
  message_filters
  pcl_conversions
  pcl_msgs
  rclcpp
  rosbag2
  sensor_msgs
  rclcpp_components
  std_msgs
  tf2
  tf2_geometry_msgs
  EIGEN3
  PCL
  BOOST
)

ament_export_include_directories(
  include
)

## Declare the pcl_ros_tf library
add_library(pcl_ros_tf src/transforms.cpp)
ament_target_dependencies(pcl_ros_tf
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)
target_include_directories(pcl_ros_tf PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_ros_tf ${PCL_LIBRARIES})
ament_export_libraries(pcl_ros_tf)

## Nodes
## Declare the pcl_ros_io library
add_library(pcl_ros_io
   src/pcl_ros/io/bag_io.cpp
   src/pcl_ros/io/concatenate_data.cpp
   src/pcl_ros/io/concatenate_fields.cpp
   src/pcl_ros/io/io.cpp
   src/pcl_ros/io/pcd_io.cpp
)
target_link_libraries(pcl_ros_io pcl_ros_tf ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
ament_export_libraries(pcl_ros_io)
ament_target_dependencies(pcl_ros_io
  "rclcpp_components"
)

# ## Declare the pcl_ros_features library
# add_library(pcl_ros_features
#   src/pcl_ros/features/feature.cpp
#   # Compilation is much faster if we include all the following CPP files in feature.cpp
#   src/pcl_ros/features/boundary.cpp
#   src/pcl_ros/features/fpfh.cpp
#   src/pcl_ros/features/fpfh_omp.cpp
#   src/pcl_ros/features/shot.cpp
#   src/pcl_ros/features/shot_omp.cpp
#   src/pcl_ros/features/moment_invariants.cpp
#   src/pcl_ros/features/normal_3d.cpp
#   src/pcl_ros/features/normal_3d_omp.cpp
#   src/pcl_ros/features/pfh.cpp
#   src/pcl_ros/features/principal_curvatures.cpp
#   src/pcl_ros/features/vfh.cpp
# )
# target_link_libraries (pcl_ros_features ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
# ament_export_libraries(pcl_ros_features)

## Declare the pcl_ros_filters library
add_library(pcl_ros_filters
#  src/pcl_ros/filters/extract_indices.cpp
  src/pcl_ros/filters/filter.cpp
#  src/pcl_ros/filters/passthrough.cpp
#  src/pcl_ros/filters/project_inliers.cpp
#  src/pcl_ros/filters/radius_outlier_removal.cpp
#  src/pcl_ros/filters/statistical_outlier_removal.cpp
#  src/pcl_ros/filters/voxel_grid.cpp
#  src/pcl_ros/filters/crop_box.cpp
)
ament_target_dependencies(pcl_ros_filters
  "rclcpp"
  "rclcpp_components"
  "rmw_implementation"
)
target_link_libraries(pcl_ros_filters pcl_ros_tf)
ament_export_libraries(pcl_ros_filters)

# Create component for crop box filter
add_library(filter_crop_box SHARED
  src/pcl_ros/filters/crop_box.cpp
)
target_link_libraries(filter_crop_box pcl_ros_filters)
rclcpp_components_register_node(filter_crop_box PLUGIN
  PLUGIN "pcl_ros::CropBox"
  EXECUTABLE filter_crop_box_node
)
install(TARGETS
  filter_crop_box
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create component for passthrough filter
add_library(filter_passthrough SHARED
  src/pcl_ros/filters/passthrough.cpp
)
target_link_libraries(filter_passthrough pcl_ros_filters)
rclcpp_components_register_node(filter_passthrough PLUGIN
  PLUGIN "pcl_ros::PassThrough"
  EXECUTABLE filter_passthrough_node
)
install(TARGETS
  filter_passthrough
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create component for radius outlier filter
add_library(filter_radius_outlier_removal SHARED
  src/pcl_ros/filters/radius_outlier_removal.cpp
)
target_link_libraries(filter_radius_outlier_removal pcl_ros_filters)
rclcpp_components_register_node(filter_radius_outlier_removal PLUGIN
  PLUGIN "pcl_ros::RadiusOutlierRemoval"
  EXECUTABLE filter_radius_outlier_removal_node
)
install(TARGETS
  filter_radius_outlier_removal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create component for voxel grid filter
add_library(filter_voxel_grid SHARED
  src/pcl_ros/filters/voxel_grid.cpp
)
target_link_libraries(filter_voxel_grid pcl_ros_filters)
rclcpp_components_register_node(filter_voxel_grid PLUGIN
  PLUGIN "pcl_ros::VoxelGrid"
  EXECUTABLE filter_voxel_grid_node
)
install(TARGETS
  filter_voxel_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# ## Declare the pcl_ros_segmentation library
# add_library (pcl_ros_segmentation
#   src/pcl_ros/segmentation/extract_clusters.cpp
#   src/pcl_ros/segmentation/extract_polygonal_prism_data.cpp
#   src/pcl_ros/segmentation/sac_segmentation.cpp
#   src/pcl_ros/segmentation/segment_differences.cpp
#   src/pcl_ros/segmentation/segmentation.cpp
# )
# target_link_libraries(pcl_ros_segmentation pcl_ros_tf ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
# class_loader_hide_library_symbols(pcl_ros_segmentation)
# 
# ## Declare the pcl_ros_surface library
# add_library (pcl_ros_surface
#   src/pcl_ros/surface/surface.cpp
#   # Compilation is much faster if we include all the following CPP files in surface.cpp
#   src/pcl_ros/surface/convex_hull.cpp
#   src/pcl_ros/surface/moving_least_squares.cpp
# )
# target_link_libraries(pcl_ros_surface ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
# class_loader_hide_library_symbols(pcl_ros_surface)
# 
# ## register node plugins
# #rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::PclNode")
# 
# ## Tools
# 
# add_executable(pcd_to_pointcloud tools/pcd_to_pointcloud.cpp)
# ament_target_dependencies(pcd_to_pointcloud
#   "pcl_conversions"
#   "rclcpp"
#   "sensor_msgs"
# )
# target_link_libraries(pcd_to_pointcloud ${boost_LIBRARIES} ${PCL_LIBRARIES})
# 
# add_executable(pointcloud_to_pcd tools/pointcloud_to_pcd.cpp)
# ament_target_dependencies(pointcloud_to_pcd
#   "pcl_conversions"
#   "rclcpp"
#   "sensor_msgs"
#   "tf2_eigen"
#   "tf2_ros"
# )
# target_link_libraries(pointcloud_to_pcd ${boost_LIBRARIES} ${PCL_LIBRARIES})
# 
# add_executable(bag_to_pcd tools/bag_to_pcd.cpp)
# target_link_libraries(bag_to_pcd pcl_ros_tf ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
# 
# add_executable(convert_pcd_to_image tools/convert_pcd_to_image.cpp)
# target_link_libraries(convert_pcd_to_image ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
# 
# 
# add_executable(convert_pcd_to_image tools/convert_pcd_to_image.cpp)
# ament_target_dependencies(convert_pcd_to_image
#   "pcl_conversions"
#   "rclcpp"
#   "sensor_msgs"
# )
# target_link_libraries(convert_pcd_to_image ${boost_LIBRARIES} ${PCL_LIBRARIES})
# 
# #add_executable(convert_pointcloud_to_image tools/convert_pointcloud_to_image.cpp)
# #target_link_libraries(convert_pointcloud_to_image ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
# 
# ## Downloads
# 
# #catkin_download(table_scene_lms400.pcd http://download.ros.org/data/pcl/table_scene_lms400.pcd
# #  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/samples/data
# #  MD5 546b5b4822fb1de21b0cf83d41ad6683
# #)
# #add_custom_target(download ALL DEPENDS download_extra_data)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #add_rostest_gtest(test_tf_message_filter_pcl tests/test_tf_message_filter_pcl.launch src/test/test_tf_message_filter_pcl.cpp)
  #target_link_libraries(test_tf_message_filter_pcl ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  #add_rostest(samples/pcl_ros/features/sample_normal_3d.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/filters/sample_statistical_outlier_removal.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/filters/sample_voxel_grid.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/segmentation/sample_extract_clusters.launch ARGS gui:=false)
  #add_rostest(samples/pcl_ros/surface/sample_convex_hull.launch ARGS gui:=false)
endif(BUILD_TESTING)


install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS
    pcl_ros_tf
    pcl_ros_io
#    pcl_ros_features
    pcl_ros_filters
#    pcl_ros_surface
#    pcl_ros_segmentation
#    pcd_to_pointcloud
#    pointcloud_to_pcd
#    bag_to_pcd
#    convert_pcd_to_image
#    convert_pointcloud_to_image
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY plugins samples
  DESTINATION share/${PROJECT_NAME})

ament_package()
