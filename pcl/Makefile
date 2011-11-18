all: installed

TARBALL = build/pcl-1.1.2.tar.gz
TARBALL_URL = https://code.ros.org/svn/release/download/thirdparty/pcl/pcl-1.1.2.tar.gz
SOURCE_DIR = build/pcl
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

#SVN_DIR = build/pcl_trunk
## Developers, please use this URL:
#SVN_URL = http://svn.pointclouds.org/pcl/branches/pcl-1.x
##SVN_PATCH = 
#SVN_REVISION=-r1806
#include $(shell rospack find mk)/svn_checkout.mk

installed: $(SOURCE_DIR)/unpacked
	mkdir -p msg/build && cd msg/build && cmake ../.. && make && cd -
	cd $(SOURCE_DIR) && mkdir -p build && cd build && \
	rm -rf ../common/include/sensor_msgs ../common/include/std_msgs \
	../common/include/pcl/ModelCoefficients.h ../common/include/pcl/PointIndices.h ../common/include/pcl/PolygonMesh.h ../common/include/pcl/Vertices.h && \
	export CPATH=`rospack find std_msgs`/msg_gen/cpp/include:`rospack find sensor_msgs`/msg_gen/cpp/include:`rospack find roscpp_serialization`/include:`rospack find cpp_common`/include:`rospack find rostime`/include:`rospack find roscpp_traits`/include:`rospack find roscpp`/include:`rospack find rosconsole`/include:`rospack find std_msgs`/include:`rospack find sensor_msgs`/include && \
	export LD_LIBRARY_PATH=`rospack find std_msgs`/lib:`rospack find sensor_msgs`/lib:`rospack find roscpp_serialization`/lib:`rospack find cpp_common`/lib:`rospack find rostime`/lib:`rospack find roscpp_traits`/lib:`rospack find roscpp`/lib:`rospack find rosconsole`/lib:`rospack find std_msgs`/lib:`rospack find sensor_msgs`/lib:$$LD_LIBRARY_PATH && \
	cmake -DCMAKE_INSTALL_PREFIX=`pwd`/../../.. \
				-DCMAKE_BUILD_TYPE=Release \
				-DUSE_ROS=ON \
          -DFLANN_INCLUDE_DIR=`rospack find flann`/include \
          -DFLANN_LIBRARY=`rospack find flann`/lib/libflann.so \
          -DCMINPACK_INCLUDE_DIR=`rospack find cminpack`/include \
          -DCMINPACK_LIBRARY=`rospack find cminpack`/lib/libcminpack.so \
	-DBUILD_TESTS=OFF \
	-DBUILD_OPENNI=ON \
	-DBUILD_apps=OFF \
	-DBUILD_common=ON \
	-DBUILD_features=ON \
	-DBUILD_filters=ON \
	-DBUILD_global_tests=OFF \
	-DBUILD_io=ON \
	-DBUILD_octree=ON \
	-DBUILD_kdtree=ON \
	-DBUILD_keypoints=ON \
	-DBUILD_range_image=ON \
	-DBUILD_registration=ON \
	-DBUILD_sample_consensus=ON \
	-DBUILD_segmentation=ON \
	-DBUILD_surface=ON \
	-DBUILD_visualization=ON \
	-DCMAKE_VERBOSE_MAKEFILE=ON \
	.. && \
	make ${ROS_PARALLEL_JOBS} install
	touch installed

cleaned: Makefile
	make clean
	touch cleaned

clean:
	-rm -rf $(SOURCE_DIR)/build rospack_nosubdirs patched installed include bin lib64 msg_gen src *~

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build cleaned
