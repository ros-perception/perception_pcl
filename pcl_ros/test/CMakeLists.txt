# ---[ Tests
rosbuild_download_test_data (http://pr.willowgarage.com/data/pcl/test_io_bag.bag test_io_bag.bag 8952974b5b7058f075ebcaa91bc8cff0)
rosbuild_add_gtest (test_io test_io.cpp)
target_link_libraries (test_io pcl_ros_io)

# ---[ ROS Tests
rosbuild_add_executable (test_bagread_publish test_bagread_publish.cpp)
rosbuild_declare_test (test_bagread_publish)
target_link_libraries (test_bagread_publish pcl_ros_io)

rosbuild_add_executable (test_bagread_subscribe test_bagread_subscribe.cpp)
rosbuild_add_gtest_build_flags (test_bagread_subscribe)
rosbuild_add_rostest (${CMAKE_CURRENT_SOURCE_DIR}/bagreader.xml)

## Future tests (won't be run by Hudson)
rosbuild_add_library (pcl_ros_tests test_passing.cpp)
#rosbuild_add_rostest_future (test_passing.test)
##rosbuild_add_rostest_future 
