# iSAM Makefile
# providing shortcuts to cmake for building outside the source tree
# Michael Kaess, 2010

make = make -j 2 -C build --no-print-directory

all: build build/Makefile
	@$(make)

# the actual build directory
build:
	@mkdir -p build bin lib include

# populate the build directory
build/Makefile:
	cd build && cmake ..

.PHONY: doc
doc:
	@doxygen doc/doxygen/isam.dox

.PHONY: distclean
distclean:
	@rm -rf build doc/html lib bin release
	@find . -name CMakeFiles |xargs rm -rf # clean up in case "cmake ." was called
	@find . -name cmake_install.cmake -delete
	@find . -name CMakeCache.txt -delete


.PHONY: examples
examples:
	@$(make) examples

# default target: any target such as "clean", "example"...
# is simply passed on to the cmake-generated Makefile 
%::
	@$(make) $@
