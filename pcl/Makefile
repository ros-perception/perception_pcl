all: installed

SVN_DIR = build/pcl_trunk
# Developers, please use this URL:
SVN_URL = http://svn.pointclouds.org/pcl/trunk # For the very latest version
#SVN_URL = http://svn.pointclouds.org/pcl/branches/pcl-1.4.x # The most recently released version
#SVN_PATCH = 
#SVN_REVISION=-r
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) cleaned
	mkdir -p msg/build && cd msg/build && cmake ../.. && make && cd -
	cd $(SVN_DIR) && mkdir -p build && cd build && \
	rm -rf ../common/include/sensor_msgs ../common/include/std_msgs \
	../common/include/pcl/ModelCoefficients.h ../common/include/pcl/PointIndices.h ../common/include/pcl/PolygonMesh.h ../common/include/pcl/Vertices.h && \
	export CPATH="`rospack cflags-only-I sensor_msgs`:`rospack cflags-only-I roscpp_serialization`:`rospack cflags-only-I cpp_common`:`rospack cflags-only-I rostime`:`rospack cflags-only-I roscpp_traits`:`rospack cflags-only-I roscpp`:`rospack cflags-only-I rosconsole`:`rospack cflags-only-I std_msgs`:`rospack cflags-only-I sensor_msgs`:`rospack find pcl`/msg_gen/cpp/include:$$CPATH" && \
	export LD_LIBRARY_PATH="`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:`rospack libs-only-L roscpp_serialization`:`rospack libs-only-L cpp_common`:`rospack libs-only-L rostime`:`rospack libs-only-L roscpp_traits`:`rospack libs-only-L roscpp`:`rospack libs-only-L rosconsole`:`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:$$LD_LIBRARY_PATH" && \
	export LIBRARY_PATH="`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:`rospack libs-only-L roscpp_serialization`:`rospack libs-only-L cpp_common`:`rospack libs-only-L rostime`:`rospack libs-only-L roscpp_traits`:`rospack libs-only-L roscpp`:`rospack libs-only-L rosconsole`:`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:$$LIBRARY_PATH" && \
	cmake -DCMAKE_INSTALL_PREFIX=`pwd`/../../.. \
				-DCMAKE_BUILD_TYPE=Release \
				-DUSE_ROS=ON \
        -DFLANN_INCLUDE_DIR=`rospack find flann`/include \
        -DFLANN_LIBRARY=`rospack find flann`/lib/libflann.so \
	-DBUILD_TESTS=OFF \
	-DBUILD_global_tests=OFF \
	-DBUILD_CUDA=ON \
	-DBUILD_outofcore=OFF \
	-DBUILD_proctor=OFF \
	-DBUILD_simulation=OFF \
	-DCMAKE_VERBOSE_MAKEFILE=OFF \
	.. && \
	make ${ROS_PARALLEL_JOBS} install
	touch installed

cleaned: Makefile
	make clean
	touch cleaned

clean:
	-rm -rf $(SVN_DIR)/build rospack_nosubdirs patched installed include bin lib64 msg_gen src *~

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build cleaned
