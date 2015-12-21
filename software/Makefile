default_target: all

# get a list of subdirs to build by reading tobuild.txt
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)
TESTDIRS:=$(shell grep -v "^\#" totest.txt)

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to two parent directories for a 'build' directory.
#   Otherwise, use ./build.
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX=$(shell for pfx in ./ .. ../..; do d=`pwd`/$$pfx/build; \
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
endif

export BUILD_PREFIX

# build quietly by default.  For a verbose build, run "make VERBOSE=1"
$(VERBOSE).SILENT:

all: 
	if [ -e build/externals.md5 ] && [ "`cat build/externals.md5`" = "`cat externals/cmake/externals.cmake | md5sum`" ]; then \
		echo "Externals are up to date"; \
	else \
		echo "Externals are out of date!"; \
		echo "Please compile externals first."; \
		exit 1; \
	fi;
	@[ -d $(BUILD_PREFIX) ] || mkdir -p $(BUILD_PREFIX) || exit 1
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done
	@# Place additional commands here if you have any

test:	all
	@for subdir in $(TESTDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir test || exit 2; \
	done
	@# Place additional commands here if you have any


clean:
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
	@# Place additional commands here if you have any
