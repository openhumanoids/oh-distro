
SUBDIRS = osrf_msgs atlas_gazebo_msgs keyboard_teleop keyboard_driving multisense_command driving_interface sandia-hand/ros/sandia_hand_msgs/ sandia-hand/ros/sandia_hand_driver/ irobot-hand translators

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
