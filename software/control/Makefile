
# todo:
#   fix .matlabroot assumption below (this makefile will get confused if make has never been called in drake)
#   support debug mode (and pass it through properly to drake)

default_target: all

# get a list of subdirs to build by reading tobuild.txt
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

# drake-related build variables
#PKG_CONFIG_PATH := ${PKG_CONFIG_PATH}:$(shell pwd)/drake/thirdParty

# NOTE:  this will fail if somebody has never run configure in drake, but the dependencies get more complicated
MATLABROOT = $(strip $(shell cat drake/.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)
MATLABCPU = $(shell cat drake/.matlabcpu)

MAPS_FLAGS = $(subst -pthread,,$(shell pkg-config --cflags-only-I --libs maps eigen3 lcm opencv))
QP_FLAGS = $(subst -pthread,,$(shell export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:./drake/thirdParty/; pkg-config --cflags --libs eigen3 gurobi))
#BULLET_FLAGS = $(shell pkg-config --cflags --libs-only-L bullet) -DBULLET_COLLISION -lBulletCollision -lLinearMath
BULLET_FLAGS =

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to three parent directories for a 'build' directory.
#   Otherwise, use ./build.
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX=$(shell for pfx in ./ .. ../.. ../../..; do d=`pwd`/$$pfx/build; \
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
SRC_BUILD_PREFIX=$(shell for pfx in ./ .. ../.. ../../..; do d=`pwd`/src/../$$pfx/build; \
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
endif

.PHONY : drake

ifeq ($(MATLABCPU),maci64)
MEXFILES = $(BUILD_PREFIX)/matlab/QPControllermex.$(MEXEXT)
else
MEXFILES = $(BUILD_PREFIX)/matlab/QPControllermex.$(MEXEXT) \
	$(BUILD_PREFIX)/matlab/HeightMapWrapper.$(MEXEXT) \
	$(BUILD_PREFIX)/matlab/bot_timestamp_now.$(MEXEXT)

endif

JAVAFILES := $(wildcard src/*.java)
CLASSFILES = $(JAVAFILES:%.java=%.class) 
SRCCLASSFILES = $(CLASSFILES:src/%=%)
CLASSPATH = $(PWD)/drake/drake.jar:/usr/local/share/java/lcm.jar:$(BUILD_PREFIX)/share/java/lcmtypes_drc_lcmtypes.jar:$(BUILD_PREFIX)/share/java/lcmtypes_bot2-core.jar:$(PWD)/src:
ifneq "$(WORKSPACE)" ""
   CLASSPATH := $(CLASSPATH):$(WORKSPACE)/software/control/drake/drake.jar
endif

# build quietly by default.  For a verbose build, run "make VERBOSE=1"
$(VERBOSE).SILENT:

all: drake $(BUILD_PREFIX)/config/drc_control_setup.m $(BUILD_PREFIX)/share/java/drc_control.jar $(MEXFILES)
	@[ -d $(BUILD_PREFIX) ] || mkdir -p $(BUILD_PREFIX) || exit 1
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done
	@# Place additional commands here if you have any

java: $(BUILD_PREFIX)/share/java/drc_control.jar

$(BUILD_PREFIX)/share/java/drc_control.jar: $(CLASSFILES)
	cd src; \
	jar -cf $(SRC_BUILD_PREFIX)/share/java/drc_control.jar $(SRCCLASSFILES); \
	cd ..; \

%.class : %.java
	javac $< -cp $(CLASSPATH); \

drake : $(BUILD_PREFIX)/config/drc_control_setup.m
	$(MAKE) -C drake all || exit 2;

$(BUILD_PREFIX)/config/drc_control_setup.m : write_drc_control_setup.m
	matlab -nosplash -nodesktop -r "setenv('BUILD_PREFIX','$(BUILD_PREFIX)'); write_drc_control_setup;exit";

ifeq ($(MATLABCPU),maci64)

$(BUILD_PREFIX)/matlab/QPControllermex.$(MEXEXT) : drake src/QPControllermex.cpp
	$(MEX) -O src/QPControllermex.cpp \
		-outdir $(BUILD_PREFIX)/matlab \
		-largeArrayDims \
		-Idrake/systems/plants drake/systems/plants/RigidBodyManipulator.o \
		$(QP_FLAGS)

else

$(BUILD_PREFIX)/matlab/HeightMapWrapper.$(MEXEXT) : src/HeightMapWrapper.cpp src/mexmaps/ViewClientWrapper.cpp src/mexmaps/FillMethods.cpp src/mexmaps/MapLib.cpp
	$(MEX) -O -v src/HeightMapWrapper.cpp src/mexmaps/ViewClientWrapper.cpp src/mexmaps/FillMethods.cpp src/mexmaps/MapLib.cpp \
		-outdir $(BUILD_PREFIX)/matlab \
		$(MAPS_FLAGS)

$(BUILD_PREFIX)/matlab/bot_timestamp_now.$(MEXEXT) : src/bot_timestamp_now.cpp
	$(MEX) -O -v src/bot_timestamp_now.cpp \
		-outdir $(BUILD_PREFIX)/matlab \
		$(MAPS_FLAGS)

$(BUILD_PREFIX)/matlab/QPControllermex.$(MEXEXT) : drake src/QPControllermex.cpp src/mexmaps/ViewClientWrapper.cpp src/mexmaps/FillMethods.cpp src/mexmaps/MapLib.cpp
	$(MEX) -O src/QPControllermex.cpp src/mexmaps/ViewClientWrapper.cpp src/mexmaps/FillMethods.cpp src/mexmaps/MapLib.cpp \
		-outdir $(BUILD_PREFIX)/matlab \
		-largeArrayDims \
		-Idrake/systems/plants drake/systems/plants/RigidBodyManipulator.o \
		-DUSE_MAPS $(MAPS_FLAGS) \
		$(BULLET_FLAGS) \
		$(QP_FLAGS)

endif

clean:
	-rm $(CLASSFILES)
	-rm $(BUILD_PREFIX)/share/java/drc_control.jar 	
	-rm $(MEXFILES)
	$(MAKE) -C drake clean;
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
	@# Place additional commands here if you have any


