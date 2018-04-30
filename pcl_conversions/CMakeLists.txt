cmake_minimum_required(VERSION 2.8.3)
project(pcl_conversions)

find_package(catkin REQUIRED COMPONENTS)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_msgs roscpp sensor_msgs std_msgs
  DEPENDS EIGEN3 PCL
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roscpp pcl_msgs sensor_msgs std_msgs)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

  catkin_add_gtest(test_pcl_conversions test/test_pcl_conversions.cpp)
  target_link_libraries(test_pcl_conversions ${catkin_LIBRARIES})
endif()
