
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

SUBDIRS = osrf_msgs atlas_gazebo_msgs keyboard_teleop keyboard_driving multisense_command driving_interface sandia-hand/ros/sandia_hand_msgs/ sandia-hand/ros/sandia_hand_driver/ irobot-hand/Software/2013_08_07_full/handle/handle_msgs irobot-hand/Software/2013_08_07_full/handle/handle_lib irobot-hand/Software/2013_08_07_full/handle/handle_ros translators

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
