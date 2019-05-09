cmake_minimum_required(VERSION 3.5)
project(pcl_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)
find_package(Boost REQUIRED)

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

# There is a bug in the Ubuntu Artful (17.10) version of the VTK package,
# where it includes /usr/include/*-linux-gnu/freetype2 in the include
# directories (which doesn't exist).  This filters down to the PCL_INCLUDE_DIRS,
# and causes downstream projects trying to use these libraries to fail to
# configure properly.  Here we remove those bogus entries so that downstream
# consumers of this package succeed.
if(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")
  foreach(item ${PCL_INCLUDE_DIRS})
    string(REGEX MATCH "/usr/include/.*-linux-gnu/freetype2" item ${item})
    if(item)
      list(REMOVE_ITEM PCL_INCLUDE_DIRS ${item})
    endif()
  endforeach()
endif()

find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)


## Add include directories
include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${pcl_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

## Declare the ament package
ament_export_dependencies(
  message_filters
  pcl_conversions
  pcl_msgs
  rclcpp
  rosbag2
  sensor_msgs
  std_msgs
  tf2
  EIGEN3
  PCL
  BOOST
)

ament_export_libraries(
  pcl_ros_filters
  pcl_ros_io
  pcl_ros_tf
)

ament_export_include_directories(
  include
)

## Declare the pcl_ros_tf library
add_library(pcl_ros_tf src/transforms.cpp)
target_link_libraries(pcl_ros_tf ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})

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
class_loader_hide_library_symbols(pcl_ros_io)

## Declare the pcl_ros_features library
add_library(pcl_ros_features
  src/pcl_ros/features/feature.cpp
  # Compilation is much faster if we include all the following CPP files in feature.cpp
  src/pcl_ros/features/boundary.cpp
  src/pcl_ros/features/fpfh.cpp
  src/pcl_ros/features/fpfh_omp.cpp
  src/pcl_ros/features/shot.cpp
  src/pcl_ros/features/shot_omp.cpp
  src/pcl_ros/features/moment_invariants.cpp
  src/pcl_ros/features/normal_3d.cpp
  src/pcl_ros/features/normal_3d_omp.cpp
  src/pcl_ros/features/pfh.cpp
  src/pcl_ros/features/principal_curvatures.cpp
  src/pcl_ros/features/vfh.cpp
)
target_link_libraries (pcl_ros_features ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(pcl_ros_features)


## Declare the pcl_ros_filters library
add_library(pcl_ros_filters
  src/pcl_ros/filters/extract_indices.cpp
  src/pcl_ros/filters/filter.cpp
  src/pcl_ros/filters/passthrough.cpp
  src/pcl_ros/filters/project_inliers.cpp
  src/pcl_ros/filters/radius_outlier_removal.cpp
  src/pcl_ros/filters/statistical_outlier_removal.cpp
  src/pcl_ros/filters/voxel_grid.cpp
  src/pcl_ros/filters/crop_box.cpp
)
target_link_libraries(pcl_ros_filters pcl_ros_tf ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(pcl_ros_filters)

## Declare the pcl_ros_segmentation library
add_library (pcl_ros_segmentation
  src/pcl_ros/segmentation/extract_clusters.cpp
  src/pcl_ros/segmentation/extract_polygonal_prism_data.cpp
  src/pcl_ros/segmentation/sac_segmentation.cpp
  src/pcl_ros/segmentation/segment_differences.cpp
  src/pcl_ros/segmentation/segmentation.cpp
)
target_link_libraries(pcl_ros_segmentation pcl_ros_tf ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(pcl_ros_segmentation)

## Declare the pcl_ros_surface library
add_library (pcl_ros_surface
  src/pcl_ros/surface/surface.cpp
  # Compilation is much faster if we include all the following CPP files in surface.cpp
  src/pcl_ros/surface/convex_hull.cpp
  src/pcl_ros/surface/moving_least_squares.cpp
)
target_link_libraries(pcl_ros_surface ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(pcl_ros_surface)

## register node plugins
rclcpp_register_node_plugins(${PROJECT_NAME} "${PROJECT_NAME}::PclNode")

## Tools

add_executable(pcd_to_pointcloud tools/pcd_to_pointcloud.cpp)
target_link_libraries(pcd_to_pointcloud ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pointcloud_to_pcd tools/pointcloud_to_pcd.cpp)
target_link_libraries(pointcloud_to_pcd ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES}${PCL_LIBRARIES})

add_executable(bag_to_pcd tools/bag_to_pcd.cpp)
target_link_libraries(bag_to_pcd pcl_ros_tf ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})

add_executable(convert_pcd_to_image tools/convert_pcd_to_image.cpp)
target_link_libraries(convert_pcd_to_image ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})

add_executable(convert_pointcloud_to_image tools/convert_pointcloud_to_image.cpp)
target_link_libraries(convert_pointcloud_to_image ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${PCL_LIBRARIES})

## Downloads

#catkin_download(table_scene_lms400.pcd http://download.ros.org/data/pcl/table_scene_lms400.pcd
#  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/samples/data
#  MD5 546b5b4822fb1de21b0cf83d41ad6683
#)
#add_custom_target(download ALL DEPENDS download_extra_data)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(gtest REQUIRED)
  # add_rostest_gtest(test_tf_message_filter_pcl tests/test_tf_message_filter_pcl.launch src/test/test_tf_message_filter_pcl.cpp)
  # target_link_libraries(test_tf_message_filter_pcl ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  # add_rostest(samples/pcl_ros/features/sample_normal_3d.launch ARGS gui:=false)
  # add_rostest(samples/pcl_ros/filters/sample_statistical_outlier_removal.launch ARGS gui:=false)
  # add_rostest(samples/pcl_ros/filters/sample_voxel_grid.launch ARGS gui:=false)
  # add_rostest(samples/pcl_ros/segmentation/sample_extract_clusters.launch ARGS gui:=false)
  # add_rostest(samples/pcl_ros/surface/sample_convex_hull.launch ARGS gui:=false)
endif(BUILD_TESTING)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS
    pcl_ros_tf
    pcl_ros_io
    pcl_ros_features
    pcl_ros_filters
    pcl_ros_surface
    pcl_ros_segmentation
    pcd_to_pointcloud
    pointcloud_to_pcd
    bag_to_pcd
    convert_pcd_to_image
    convert_pointcloud_to_image
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY plugins samples
  DESTINATION share/${PROJECT_NAME})

ament_package()
