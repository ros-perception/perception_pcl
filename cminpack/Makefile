all: installed

VER = 1.1.3
TARBALL       = build/cminpack-$(VER).tar.gz
TARBALL_URL   = http://pr.willowgarage.com/downloads/cminpack-$(VER).tar.gz
UNPACK_CMD    = tar xfz
SOURCE_DIR    = build/cminpack-$(VER)
#TARBALL_PATCH = cminpack.pc.patch
MD5SUM_FILE   = cminpack-$(VER).tar.gz.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

ifeq ($(shell uname -s 2>/dev/null),Darwin)
	LIBEXT=dylib
else
	LIBEXT=so
endif


installed: $(SOURCE_DIR)/unpacked Makefile
	echo "Building CMinpack..."
	cd $(SOURCE_DIR) && mkdir -p build && cd build && cmake -DUSE_FPIC=ON -DSHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMINPACK_LIB_INSTALL_DIR="." .. && make && cd ../../../../
	mkdir -p include lib
	cp $(SOURCE_DIR)/*.h include
	cp -P $(SOURCE_DIR)/build/libcminpack.* lib
	touch installed
clean: wipe
wipe:
	rm -rf build installed lib include
