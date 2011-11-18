cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init ()
rosbuild_add_boost_directories ()
#add_definitions (-Wall -O3 -funroll-loops)
add_definitions (-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (src)

# Enable dynamic_reconfigure
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

# ---[ Point Cloud Library - Transforms
rosbuild_add_library (pcl_ros_tf src/pcl_ros/transforms.cpp)
rosbuild_add_compile_flags (pcl_ros_tf ${SSE_FLAGS})

# ---[ Point Cloud Library - IO
rosbuild_add_library (pcl_ros_io
                      src/pcl_ros/io/io.cpp
                      src/pcl_ros/io/concatenate_fields.cpp
                      src/pcl_ros/io/concatenate_data.cpp
                      src/pcl_ros/io/pcd_io.cpp
                      src/pcl_ros/io/bag_io.cpp
                     )
rosbuild_add_compile_flags (pcl_ros_io ${SSE_FLAGS})
rosbuild_link_boost (pcl_ros_io system)
target_link_libraries (pcl_ros_io pcl_ros_tf)

# ---[ PCL ROS - Features
rosbuild_add_library (pcl_ros_features
                      src/pcl_ros/features/feature.cpp
                      # Compilation is much faster if we include all the following CPP files in feature.cpp
                      src/pcl_ros/features/boundary.cpp
                      src/pcl_ros/features/fpfh.cpp
                      src/pcl_ros/features/fpfh_omp.cpp
                      src/pcl_ros/features/moment_invariants.cpp
                      src/pcl_ros/features/normal_3d.cpp
                      src/pcl_ros/features/normal_3d_omp.cpp
                      src/pcl_ros/features/pfh.cpp
                      src/pcl_ros/features/principal_curvatures.cpp
                      src/pcl_ros/features/vfh.cpp
                     )
rosbuild_add_compile_flags (pcl_ros_features ${SSE_FLAGS})
rosbuild_add_openmp_flags (pcl_ros_features)

# ---[ Point Cloud Library - Surface
rosbuild_add_library (pcl_ros_surface
                      src/pcl_ros/surface/surface.cpp
                      # Compilation is much faster if we include all the following CPP files in surface.cpp
                      src/pcl_ros/surface/convex_hull.cpp
                      src/pcl_ros/surface/moving_least_squares.cpp
                     )
rosbuild_add_compile_flags (pcl_ros_surface ${SSE_FLAGS})

# ---[ PCL ROS - Filters
rosbuild_add_library (pcl_ros_filters
                      src/pcl_ros/filters/filter.cpp
                      src/pcl_ros/filters/passthrough.cpp
                      src/pcl_ros/filters/project_inliers.cpp
                      src/pcl_ros/filters/extract_indices.cpp
                      src/pcl_ros/filters/radius_outlier_removal.cpp
                      src/pcl_ros/filters/statistical_outlier_removal.cpp
                      src/pcl_ros/filters/voxel_grid.cpp
                     )
rosbuild_add_compile_flags (pcl_ros_filters ${SSE_FLAGS})
target_link_libraries (pcl_ros_filters pcl_ros_tf)

# ---[ Point Cloud Library - Segmentation
rosbuild_add_library (pcl_ros_segmentation
                      src/pcl_ros/segmentation/segmentation.cpp
                      src/pcl_ros/segmentation/segment_differences.cpp
                      src/pcl_ros/segmentation/extract_clusters.cpp
                      src/pcl_ros/segmentation/extract_polygonal_prism_data.cpp
                      src/pcl_ros/segmentation/sac_segmentation.cpp
                     )
rosbuild_add_compile_flags (pcl_ros_segmentation ${SSE_FLAGS})
target_link_libraries (pcl_ros_segmentation pcl_segmentation pcl_ros_tf)
rosbuild_link_boost (pcl_ros_segmentation system filesystem)

add_subdirectory (src/tools)
add_subdirectory (test)

