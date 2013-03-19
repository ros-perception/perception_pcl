all: installed

GIT_DIR = build/pcl_trunk
# Developers, please use this URL:
GIT_URL = https://github.com/PointCloudLibrary/pcl.git # For the very latest version
#GIT_PATCH = 
GIT_REVISION= 6eb5ea26d0ba64e7308ef2b725aaefbdfaa734f0 # Update this when doing a new release!

PCL_VERSION = 17

REPLACEMENTS = replace_in_files "./CMakeLists.txt" '\-Wold-style-cast' '\-Wno-old-style-cast -Wno-unused-parameter -Wno-conversion' && \
	replace_in_files "$$source_files" 'namespace pcl' 'namespace pcl$(PCL_VERSION)' && \
	replace_in_files "$$source_files" 'pcl::' 'pcl$(PCL_VERSION)::' && \
	replace_in_files "$$source_files" 'pcl_gpu::' 'pcl$(PCL_VERSION)_gpu::' && \
	replace_in_files "$$source_files" 'pcl_cuda::' 'pcl$(PCL_VERSION)_cuda::' && \
	replace_in_files "$$source_files" 'include <pcl/' 'include <pcl$(PCL_VERSION)/' && \
	replace_in_files "$$source_files" 'include "pcl/' 'include "pcl$(PCL_VERSION)/' && \
	replace_in_files "$$source_files" 'pcl_round' 'pcl$(PCL_VERSION)_round' && \
	replace_in_files "$$source_files" 'pcl_lrint' 'pcl$(PCL_VERSION)_lrint' && \
	replace_in_files "$$source_files" 'pcl_sleep' 'pcl$(PCL_VERSION)_sleep' && \
	replace_in_files "$$source_files" 'PCLAPI' 'PCL$(PCL_VERSION)API' && \
	replace_in_files "$$source_files" 'aligned_malloc' 'pcl$(PCL_VERSION)_aligned_malloc' && \
	replace_in_files "$$source_files" 'aligned_free' 'pcl$(PCL_VERSION)_aligned_free' && \
	replace_in_files "$$source_files" 'POINT_CLOUD_' 'PCL$(PCL_VERSION)_POINT_CLOUD_' && \
	replace_in_files "`find ./ -name CMakeLists.txt`" 'include/pcl' 'include/pcl$(PCL_VERSION)' && \
	replace_in_files "`find ./common/include/pcl/console/ -name *.h`" 'TERMINAL_TOOLS' 'PCL$(PCL_VERSION)_TERMINAL_TOOLS' && \
	replace_in_files "./cmake/pcl_utils.cmake" '$${INCLUDE_INSTALL_ROOT}/pcl' '$${INCLUDE_INSTALL_ROOT}/pcl$(PCL_VERSION)' && \
	replace_in_files "$$source_files" 'PCL_' 'PCL$(PCL_VERSION)_'

#include $(shell rospack find mk)/svn_checkout.mk

ifneq ($(r),)
  ifeq ($(r),head) 
    GIT_REVISION=
  else
    GIT_REVISION=$(r)
  endif
endif

ifeq ($(strip $(GIT_CMDLINE)),)
GIT_CMDLINE = git
endif

REPLACE_IN_FILES = replace_in_files_f () { \
	files=`echo $$1 | xargs grep -H "$$2" | awk {'print $$1'} | awk -F ":" {'print $$1'} | sort -n | uniq` && \
	query=$$(echo $$2 | sed -e 's/\//\\\//g') && replace=$$(echo $$3 | sed -e 's/\//\\\//g') && \
	for i in $$files; do filetime=`stat -c %y $$i` && sed -i -e s/"$$query"/"$$replace"/g $$i && touch -md "$$filetime" $$i ; done } && \
	source_files=`find ./ -iname "*.[c|h]pp" -o -name "*.h" -o -name "*.c" -o -name "*.cu" -o -name "*.cxx"`

patched: $(GIT_DIR)
ifeq ($(wildcard patched), )
	cd $(GIT_DIR) && \
	mkdir -p build && \
	mv build ./../build_temp && \
	$(REPLACE_IN_FILES) && \
	replace_in_files () { replace_in_files_f "$$1" "$$2" "$$3" ; } && \
	$(REPLACEMENTS) && \
	dirs=`find ./ -name pcl` && \
	for d in $$dirs; do mv $$d `echo $$d | sed 's/\(.*\)pcl/\1pcl$(PCL_VERSION)/'`; done && \
	mv ./../build_temp ./build
	touch patched
else
	@echo "$(GIT_DIR) is patched"
endif

undo_patch: $(GIT_DIR)
ifneq ($(wildcard patched), )
	cd $(GIT_DIR) && \
	mkdir -p build && \
	mv build ./../build_temp && \
	dirs=`find ./ -name pcl$(PCL_VERSION)` && \
	for d in $$dirs; do mv $$d `echo $$d | sed 's/\(.*\)pcl$(PCL_VERSION)/\1pcl/'`; done && \
	$(REPLACE_IN_FILES) && \
	replace_in_files () { replace_in_files_f "$$1" "$$3" "$$2" ; } && \
	$(REPLACEMENTS) && \
	mv ./../build_temp ./build
	rm patched
else
	@echo "$(GIT_DIR) is not patched"
endif

test_patching:
	make wipe
	make installed
	make undo_patch
	make clean
	cd $(GIT_DIR) && git status

revert: 
	make undo_patch
	cd $(GIT_DIR) && git clean -f -r && git reset $(GIT_REVISION) --hard

$(GIT_DIR):
	if [ ! -d build ]; then \
	mkdir build;\
	fi
	if [ ! -d $(GIT_DIR) ]; then \
 	mkdir $(GIT_DIR); \
	fi
	$(GIT_CMDLINE) clone  $(GIT_URL) $(GIT_DIR)
	cd $(GIT_DIR) && $(GIT_CMDLINE) reset $(GIT_REVISION) --hard
ifneq ($(strip $(GIT_PATCH)),)
	cd $(GIT_DIR) && patch -p0 < ../$(GIT_PATCH)
endif

GIT_UP: 
	make undo_patch
	cd $(GIT_DIR) && $(GIT_CMDLINE) pull && $(GIT_CMDLINE) reset $(GIT_REVISION) --hard
	#TODO: this is a revert, not a update. Maybe use and TEST "checkout".
	make installed

download: $(GIT_UP)

installed: patched
	mkdir -p msg/build && cd msg/build && cmake ../.. && make && cd -
	#rm -rf include bin lib64
	cd $(GIT_DIR) && mkdir -p build && cd build && \
	rm -rf ../common/include/sensor_msgs ../common/include/std_msgs \
	../common/include/pcl$(PCL_VERSION)/ModelCoefficients.h ../common/include/pcl$(PCL_VERSION)/PointIndices.h ../common/include/pcl$(PCL_VERSION)/PolygonMesh.h ../common/include/pcl$(PCL_VERSION)/Vertices.h && \
	export CPATH="`rospack cflags-only-I sensor_msgs`:`rospack cflags-only-I roscpp_serialization`:`rospack cflags-only-I cpp_common`:`rospack cflags-only-I rostime`:`rospack cflags-only-I roscpp_traits`:`rospack cflags-only-I roscpp`:`rospack cflags-only-I rosconsole`:`rospack cflags-only-I std_msgs`:`rospack cflags-only-I sensor_msgs`:`rospack find pcl$(PCL_VERSION)`/msg_gen/cpp/include:$$CPATH" && \
	export LD_LIBRARY_PATH="`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:`rospack libs-only-L roscpp_serialization`:`rospack libs-only-L cpp_common`:`rospack libs-only-L rostime`:`rospack libs-only-L roscpp_traits`:`rospack libs-only-L roscpp`:`rospack libs-only-L rosconsole`:`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:$$LD_LIBRARY_PATH" && \
	export LIBRARY_PATH="`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:`rospack libs-only-L roscpp_serialization`:`rospack libs-only-L cpp_common`:`rospack libs-only-L rostime`:`rospack libs-only-L roscpp_traits`:`rospack libs-only-L roscpp`:`rospack libs-only-L rosconsole`:`rospack libs-only-L std_msgs`:`rospack libs-only-L sensor_msgs`:$$LIBRARY_PATH" && \
	cmake -DCMAKE_INSTALL_PREFIX=`rospack find pcl17` \
				-DCMAKE_BUILD_TYPE=Release \
				-DUSE_ROS=ON \
	-DBUILD_CUDA=ON \
	-DBUILD_GPU=ON \
	-DBUILD_OPENNI=ON \
	-DBUILD_TESTS=OFF \
	-DBUILD_apps=OFF \
	-DBUILD_examples=OFF \
	-DBUILD_outofcore=OFF \
	-DBUILD_simulation=OFF \
	-DCMAKE_VERBOSE_MAKEFILE=OFF \
	.. && \
	make ${ROS_PARALLEL_JOBS} install
	#rename the lib files and refresh manifest.xml
	cd `pwd` && \
	libs=`find ./lib -type f -name "libpcl*"` && \
	for d in $$libs; do ln -fs `echo $$d | sed 's/\.\/lib\/\(.*\)/\1/'` `echo $$d | sed 's/\(.*libpcl\)\(.*\.so\).*/\1$(PCL_VERSION)\2/'`; done && \
	lib_names= && \
	for d in $$libs; do lib_names="$$lib_names`echo $$d | sed 's/.*lib\(pcl\)\(.*\)\.so.*/ -l\1$(PCL_VERSION)\2/'`"; done && \
	if ! `grep -qi "\"-L\\$${prefix}/lib $${lib_names}\"" manifest.xml`; then \
	sed -i "s/\(<cpp.*lflags=\)\"[^\"]*\"\(.*\/>\)/\1\"-L$$\{prefix\}\/lib $${lib_names}\"\2/" manifest.xml ; \
        fi
	ln -fs lib lib64
	touch installed

cleaned: Makefile
	make clean
	touch cleaned

clean:
	-rm -rf $(GIT_DIR)/build rospack_nosubdirs installed include bin lib64 msg_gen src *~

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build cleaned patched msg/build
