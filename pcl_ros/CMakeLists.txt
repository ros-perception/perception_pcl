cmake_minimum_required(VERSION 3.5)
project(pcl_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

## Find ROS package dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_export_dependencies(
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  EIGEN3
  PCL
)

## Declare the pcl_ros_tf library
add_library(pcl_ros_tf src/transforms.cpp)
ament_target_dependencies(pcl_ros_tf
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
target_include_directories(pcl_ros_tf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_ros_tf ${PCL_LIBRARIES})
ament_export_libraries(pcl_ros_tf)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # TODO
endif(BUILD_TESTING)


install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS
    pcl_ros_tf
#    pcl_ros_io
#    pcl_ros_features
#    pcl_ros_filters
#    pcl_ros_surface
#    pcl_ros_segmentation
#    pcd_to_pointcloud
#    pointcloud_to_pcd
#    bag_to_pcd
#    convert_pcd_to_image
#    convert_pointcloud_to_image
  EXPORT export_pcl_ros
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY plugins samples
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_pcl_ros HAS_LIBRARY_TARGET)
ament_package()
