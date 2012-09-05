all: installed

SVN_DIR = build/pcl_1.6.0
# Developers, please use this URL:
SVN_URL = http://svn.pointclouds.org/pcl/tags/pcl-1.6.0 # For the very latest version

ifeq ($(strip $(SVN_CMDLINE)),)
SVN_CMDLINE = svn
endif

$(SVN_DIR):
	$(SVN_CMDLINE) co $(SVN_REVISION) $(SVN_URL) $(SVN_DIR)
ifneq ($(strip $(SVN_PATCH)),)
	cd $(SVN_DIR) && patch -p0 < ../$(SVN_PATCH)
endif
	cd $(SVN_DIR) && \
  sed -Ei 's/-Wold-style-cast/-Wno-old-style-cast -Wno-unused-parameter -Wno-conversion/g' ./CMakeLists.txt && \
	sed -Ei 's/${INCLUDE_INSTALL_ROOT}\/pcl/${INCLUDE_INSTALL_ROOT}\/pcl$(PCL_VERSION)/g' ./cmake/pcl_utils.cmake

SVN_UP: $(SVN_DIR)
	cd $(SVN_DIR) && $(SVN_CMDLINE) up $(SVN_REVISION)

download: $(SVN_UP)

installed: $(SVN_DIR) cleaned
	mkdir -p msg/build && cd msg/build && cmake ../.. && make && cd -
	cd $(SVN_DIR) && mkdir -p build && cd build && \
	rm -rf ../common/include/sensor_msgs ../common/include/std_msgs \
	../common/include/pcl$(PCL_VERSION)/ModelCoefficients.h ../common/include/pcl$(PCL_VERSION)/PointIndices.h ../common/include/pcl$(PCL_VERSION)/PolygonMesh.h ../common/include/pcl$(PCL_VERSION)/Vertices.h && \
	export CPATH="`rospack cflags-only-I sensor_msgs`:`rospack cflags-only-I roscpp_serialization`:`rospack cflags-only-I cpp_common`:`rospack cflags-only-I rostime`:`rospack cflags-only-I roscpp_traits`:`rospack cflags-only-I roscpp`:`rospack cflags-only-I rosconsole`:`rospack cflags-only-I std_msgs`:`rospack cflags-only-I sensor_msgs`:`rospack find pcl$(PCL_VERSION)`/msg_gen/cpp/include:$$CPATH" && \
	export LD_LIBRARY_PATH="`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:`rospack libs-only-L roscpp_serialization`:`rospack libs-only-L cpp_common`:`rospack libs-only-L rostime`:`rospack libs-only-L roscpp_traits`:`rospack libs-only-L roscpp`:`rospack libs-only-L rosconsole`:`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:$$LD_LIBRARY_PATH" && \
	export LIBRARY_PATH="`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:`rospack libs-only-L roscpp_serialization`:`rospack libs-only-L cpp_common`:`rospack libs-only-L rostime`:`rospack libs-only-L roscpp_traits`:`rospack libs-only-L roscpp`:`rospack libs-only-L rosconsole`:`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:$$LIBRARY_PATH" && \
	cmake -DCMAKE_INSTALL_PREFIX=`pwd`/../../.. \
				-DCMAKE_BUILD_TYPE=Release \
				-DUSE_ROS=ON \
	-DBUILD_OPENNI=ON \
	-DBUILD_TESTS=OFF \
	-DBUILD_apps=OFF \
	-DBUILD_examples=OFF \
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
	rm -rf build cleaned msg/build
