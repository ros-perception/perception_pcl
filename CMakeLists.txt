cmake_minimum_required(VERSION 2.8)

project(perception_pcl)

# Deal with catkin
find_package(catkin REQUIRED roscpp sensor_msgs)
find_package(PCL)
find_package(eigen)

# deal with ROS
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_stack()

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include 
  LIBRARIES pcl_ros_tf
  DEPENDS  PCL eigen 
)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${eigen_INCLUDE_DIRS})
include_directories(include)


# ---[ Point Cloud Library - Transforms
add_library (pcl_ros_tf SHARED src/transforms.cpp)
target_link_libraries(pcl_ros_tf ${PCL_LIBS})

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS pcl_ros_tf
        RUNTIME DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
)
