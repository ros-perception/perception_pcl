cmake_minimum_required(VERSION 3.5)
project(pcl_conversions)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${pcl_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

# Add gtest based cpp test target
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}-test test/test_pcl_conversions.cpp)
  ament_target_dependencies(${PROJECT_NAME}-test
    "pcl_msgs"
    "rclcpp"
    "sensor_msgs"
    "std_msgs"
  )
  target_link_libraries(${PROJECT_NAME}-test ${Boost_LIBRARIES})
endif()

ament_export_include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${pcl_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

ament_export_dependencies()

ament_package()
