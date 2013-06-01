cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include(ExternalProject)

SET(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/pcl_trunk/src")

INSTALL(CODE "MESSAGE(\"make install\")")

ExternalProject_Add( pcl_trunk
  PREFIX pcl_trunk
  GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl.git
  GIT_TAG e386f35d506d5b2e058900004aca2366b9585147
  DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/pcl_trunk"
  SOURCE_DIR "${SOURCE_DIR}"
  BUILD_COMMAND .
  PATCH_COMMAND rm -rf ${SOURCE_DIR}/common/include/sensor_msgs ${SOURCE_DIR}/common/include/std_msgs ${SOURCE_DIR}/common/include/pcl/ModelCoefficients.h ${SOURCE_DIR}/common/include/pcl/PointIndices.h ${SOURCE_DIR}/common/include/pcl/PolygonMesh.h ${SOURCE_DIR}/common/include/pcl/Vertices.h
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
             -DUSE_ROS=ON
             -DBUILD_CUDA=ON
             -DBUILD_GPU=ON
             -DBUILD_OPENNI=ON
             -DBUILD_TESTS=OFF
             -DBUILD_apps=OFF
             -DBUILD_examples=OFF
             -DBUILD_outofcore=OFF
             -DBUILD_simulation=OFF
             -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
