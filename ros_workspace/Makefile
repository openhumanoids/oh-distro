
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

SUBDIRS = atlas_gazebo_msgs translators keyboard_teleop keyboard_driving multisense_command driving_interface

all: $(SUBDIRS)
	echo Making MITs ROS DRC repo
	@for subdir in $(SUBDIRS); do \
		$(MAKE) -C $$subdir ; \
	done
	echo "===Finished making MIT's ROS DRC repo===";


clean:
	@for subdir in $(SUBDIRS); do \
		$(MAKE) -C $$subdir clean; \
	done
