find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake)

add_executable(dummy_point_cloud2_publisher
  dummy_point_cloud2_publisher.cpp
)
target_link_libraries(dummy_point_cloud2_publisher
  ${PCL_LIBRARIES}
)
ament_target_dependencies(dummy_point_cloud2_publisher
  rclcpp
  pcl_conversions
  sensor_msgs
  PCL
)
set(options true false)
foreach(use_indices IN LISTS options)
  set(filter_name_pairs
    crop_box|CropBox
    extract_indices|ExtractIndices
    passthrough|PassThrough
    #project_inliers|ProjectInliers
    radius_outlier_removal|RadiusOutlierRemoval
    statistical_outlier_removal|StatisticalOutlierRemoval
    voxel_grid|VoxelGrid
  )
  foreach(pair IN LISTS filter_name_pairs)
    string(REPLACE "|" ";" pair ${pair})
    list(GET pair 0 filter_name)
    list(GET pair 1 class_name)
    ament_add_test(test_filter_${filter_name}_use_indices=${use_indices}
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "launch_test" "${CMAKE_CURRENT_SOURCE_DIR}/test_filters_launch.py"
      ENV
        DUMMY_PUBLISHER=$<TARGET_FILE:dummy_point_cloud2_publisher>
        FILTER_NAME=${filter_name}
        CLASS_NAME=${class_name}
        EXTRA_ROS_ARGS=-p\ use_indices:=${use_indices}
    )
  endforeach()
endforeach()
# slitely different test for ProjectInliers
# indecies and model are required to work properly
# use_indices is not implemented
ament_add_test(test_filter_project_inliers
GENERATE_RESULT_FOR_RETURN_CODE_ZERO
COMMAND "launch_test" "${CMAKE_CURRENT_SOURCE_DIR}/test_filters_launch.py"
ENV
  DUMMY_PUBLISHER=$<TARGET_FILE:dummy_point_cloud2_publisher>
  FILTER_NAME=project_inliers
  CLASS_NAME=ProjectInliers
  EXTRA_ROS_ARGS=-p\ model_type:=0
)
