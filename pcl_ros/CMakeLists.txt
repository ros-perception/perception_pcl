cmake_minimum_required(VERSION 2.8)
project(pcl_ros)

# Deal with catkin
find_package(catkin REQUIRED roscpp sensor_msgs tf)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(Eigen)
find_package(PCL)

# deal with ROS
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

catkin_package(DEPENDS Eigen PCL roscpp sensor_msgs tf
               INCLUDE_DIRS include 
               LIBRARIES pcl_ros_tf
)

# ---[ Point Cloud Library - Transforms
add_library (pcl_ros_tf SHARED src/transforms.cpp)
target_link_libraries(pcl_ros_tf ${PCL_LIBS} ${Boost_LIBS} ${catkin_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS pcl_ros_tf
        RUNTIME DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
)
