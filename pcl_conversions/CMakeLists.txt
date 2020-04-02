cmake_minimum_required(VERSION 3.0.2)
project(pcl_conversions)

find_package(catkin REQUIRED COMPONENTS)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

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

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_msgs roscpp sensor_msgs std_msgs
  DEPENDS EIGEN3 PCL
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(Boost REQUIRED COMPONENTS iostreams)
  find_package(catkin REQUIRED COMPONENTS roscpp pcl_msgs sensor_msgs std_msgs)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

  catkin_add_gtest(test_pcl_conversions test/test_pcl_conversions.cpp)
  target_link_libraries(test_pcl_conversions ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
