cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include(ExternalProject)

SET(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/pcl_trunk/src")

INSTALL(CODE "MESSAGE(\"make install\")")

ExternalProject_Add( pcl_trunk
  PREFIX pcl_trunk
  GIT_REPOSITORY https://github.com/PointCloudLibrary/pcl.git
  GIT_TAG cd902a41c84921d6056ee0eacf9ada4591b9d1e3
  DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/pcl_trunk"
  SOURCE_DIR "${SOURCE_DIR}"
  BUILD_COMMAND .
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
             -DBUILD_CUDA=ON
             -DBUILD_GPU=ON
             -DBUILD_OPENNI=ON
             -DBUILD_TESTS=OFF
             -DBUILD_apps=OFF
             -DBUILD_outofcore=OFF
             -DBUILD_simulation=OFF
             -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
