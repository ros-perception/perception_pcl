cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init ()
rosbuild_add_boost_directories ()
add_definitions (-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (src)

# Uses Eigen
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

# ---[ Point Cloud Library - Transforms
rosbuild_add_library (pcl17_ros_tf src/pcl17_ros/transforms.cpp)
rosbuild_add_compile_flags (pcl17_ros_tf ${SSE_FLAGS})
