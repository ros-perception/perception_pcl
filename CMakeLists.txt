cmake_minimum_required(VERSION 2.8)

project(perception_pcl)

# Deal with catkin
find_package(catkin REQUIRED)
catkin_stack()

# deal with ROS
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(PCL)
find_package(eigen)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${eigen_INCLUDE_DIRS})
include_directories(include)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include 
  LIBRARIES pcl_ros_tf
  DEPENDS  PCL eigen 
)

# ---[ Point Cloud Library - Transforms
add_library (pcl_ros_tf src/transforms.cpp)

