cmake_minimum_required(VERSION 2.8.3)
project(pcl_conversions)

find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs)

find_package(PCL REQUIRED COMPONENTS COMMON)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs std_msgs
  DEPENDS pcl
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Add gtest based cpp test target
catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl_conversions.cpp)
target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
