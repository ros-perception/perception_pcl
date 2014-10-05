cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_laserscan)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_filters
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PointCloudToLaserScan PointCloudToLaserScanNodelet
  CATKIN_DEPENDS dynamic_reconfigure libpcl-all-dev message_filters nodelet pcl_ros roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(PointCloudToLaserScan src/PointCloudToLaserScanBase.cpp)
target_link_libraries(PointCloudToLaserScan ${catkin_LIBRARIES})

add_library(PointCloudToLaserScanNodelet src/PointCloudToLaserScanNodelet.cpp)
target_link_libraries(PointCloudToLaserScanNodelet PointCloudToLaserScan ${catkin_LIBRARIES})

add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan_node.cpp)
target_link_libraries(pointcloud_to_laserscan PointCloudToLaserScan ${catkin_LIBRARIES})

install(TARGETS PointCloudToLaserScan PointCloudToLaserScanNodelet pointcloud_to_laserscan
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
