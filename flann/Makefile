all: installed

VER = 1.6.10
TARBALL       = build/flann-$(VER)-src.zip
TARBALL_URL   = http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-$(VER)-src.zip
UNPACK_CMD    = unzip -x
SOURCE_DIR    = build/flann-$(VER)-src
TARBALL_PATCH = hist_intersect_union.patch doc.patch
MD5SUM_FILE   = flann-$(VER)-src.zip.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	echo "Building FLANN..."
	cd $(SOURCE_DIR) && cmake -DCMAKE_INSTALL_PREFIX=`pwd`/../.. -DBUILD_C_BINDINGS=true -DBUILD_PYTHON_BINDINGS=false -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_MATLAB_BINDINGS=false && make install
#	if [ `uname` = Darwin ]; then \
#		install_name_tool -id `rospack find flann`/lib/libflann_cpp.${VER}.dylib lib/libflann_cpp.${VER}.dylib; \
#		install_name_tool -id `rospack find flann`/lib/libflann.${VER}.dylib lib/libflann.${VER}.dylib; \
#	fi
	-ln -s lib64 lib
	touch installed

clean: wipe

wiped: Makefile
	make wipe
	touch wiped

wipe:
	rm -rf build installed bin doc include lib lib64 share

