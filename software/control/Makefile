default_target: all

# get a list of subdirs to build by reading tobuild.txt
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

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
PWD = $(shell pwd)

# The following line is needed to get make to always run the MATLAB make
.PHONY: matlabfoo
MATLAB = $(shell which matlab)
JAVAFILES := $(wildcard src/*.java)
CLASSFILES = $(JAVAFILES:%.java=%.class) 
SRCCLASSFILES = $(CLASSFILES:src/%=%)
CLASSPATH = $(PWD)/drake/drake.jar:/usr/local/share/java/lcm.jar:$(BUILD_PREFIX)/share/java/lcmtypes_drc_lcmtypes.jar

# build quietly by default.  For a verbose build, run "make VERBOSE=1"
$(VERBOSE).SILENT:

define build-drake
echo "Checking to see if you have MATLAB installed and in your PATH...";\
if [ -z $(MATLAB) ]; then\
	echo "You do not appear to have MATLAB installed, or it is not in your PATH. Skipping Drake build.";\
else\
	echo "You appear to have an available MATLAB, so I'll try to build Drake now...";\
	matlab -nosplash -nodesktop -r "setenv('BUILD_PREFIX','$(BUILD_PREFIX)'); make;exit";\
	echo "All done configuring and building Drake.";\
fi
endef

all: $(BUILD_PREFIX)/share/java/drc_control.jar drake
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

drake: matlabfoo $(BUILD_PREFIX)/share/java/drc_control.jar
	@$(build-drake)

clean:
	-rm $(CLASSFILES)
	-rm $(BUILD_PREFIX)/share/java/drc_control.jar 	
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
	@# Place additional commands here if you have any


