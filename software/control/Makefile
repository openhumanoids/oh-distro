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
endif

JAVAFILES := $(wildcard src/*.java)
CLASSFILES = $(JAVAFILES:%.java=%.class) 
CLASSPATH = drake/drake.jar:$(BUILD_PREFIX)/share/java/lcm.jar:$(BUILD_PREFIX)/share/java/lcmtypes_drc_lcmtypes.jar


# build quietly by default.  For a verbose build, run "make VERBOSE=1"
$(VERBOSE).SILENT:

all: drake $(BUILD_PREFIX)/share/java/drc_control.jar
	@[ -d $(BUILD_PREFIX) ] || mkdir -p $(BUILD_PREFIX) || exit 1
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done
	@# Place additional commands here if you have any

$(BUILD_PREFIX)/share/java/drc_control.jar: $(CLASSFILES)
	jar -cf $@ $(CLASSFILES) 

%.class : %.java
	javac $< -cp $(CLASSPATH)

drake:
	echo "\nBuilding drake\n";
	matlab -nosplash -nodesktop -r "make"

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

