cmake_minimum_required(VERSION 2.8)
project(pcl_msgs)

# Deal with catkin
find_package(catkin REQUIRED genmsg sensor_msgs std_msgs)

# create messages
### what's below is just to generate the messages that are manually patched into PCL
###project(pcl)
add_message_files(DIRECTORY msg
                  FILES ModelCoefficients.msg
                        PointIndices.msg
                        PolygonMesh.msg
                        Vertices.msg
)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(DEPENDS sensor_msgs std_msgs)

