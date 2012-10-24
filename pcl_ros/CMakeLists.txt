cmake_minimum_required(VERSION 2.8)
project(pcl_ros)

# Deal with catkin
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(catkin REQUIRED genmsg roscpp sensor_msgs std_msgs tf)
find_package(Eigen)
find_package(PCL)

# deal with ROS
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${Eigen_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
include_directories(include)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

catkin_package(DEPENDS Eigen PCL roscpp sensor_msgs tf
               INCLUDE_DIRS include 
               LIBRARIES pcl_ros_tf
)

# create messages
project(pcl)
add_message_files(DIRECTORY msg
                  FILES ModelCoefficients.msg
                        PointIndices.msg
                        PolygonMesh.msg
                        Vertices.msg
)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

project(pcl_ros)
# ---[ Point Cloud Library - Transforms
add_library (pcl_ros_tf SHARED src/transforms.cpp)
target_link_libraries(pcl_ros_tf ${PCL_LIBS} ${Boost_LIBS} ${catkin_LIBS})
add_dependencies(pcl_ros_tf ros_gencpp pcl_ros_copy)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS pcl_ros_tf
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
