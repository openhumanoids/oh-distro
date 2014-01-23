
SUBDIRS = atlas_gazebo_msgs keyboard_teleop keyboard_driving multisense_command driving_interface irobot-hand mit_helios_scripts translators

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
