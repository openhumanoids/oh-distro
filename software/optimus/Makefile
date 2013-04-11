DRC_ROOT=${HOME}/drc
DRC_BIN=${DRC_ROOT}/software/build/bin
ROS_BIN=/opt/ros/fuerte/bin
DEST_DIR = ${DRC_ROOT}/software/build/optimus

DRC_BINS=\
drc-viewer\
drc-image-passthrough\
drc-lidar-passthrough\
stereo-bm\

ROS_BINS=\
roslaunch

SHELL_CMDS=$(addprefix ${DEST_DIR}/, ${DRC_BINS} ${ROS_BINS})

ARGS_STR=\"$$\{@:1\}\"

default: clean dir ${DRC_BINS} ${ROS_BINS}

dir:
	@mkdir -p ${DEST_DIR}
	@ln -fs ${DRC_BIN}/bot-procman-deputy ${DEST_DIR}/

${DRC_BINS}:
	@echo "Creating $@"
	@echo "#!/bin/bash" > ${DEST_DIR}/$@
	@echo "vblank_mode=0 primusrun ${DRC_BIN}/$@" ${ARGS_STR} >> ${DEST_DIR}/$@
	@chmod 755 ${DEST_DIR}/$@

${ROS_BINS}:
	@echo "Creating $@"
	@echo "#!/bin/bash" > ${DEST_DIR}/$@
	@echo "vblank_mode=0 primusrun ${ROS_BIN}/$@" ${ARGS_STR} >> ${DEST_DIR}/$@
	@chmod 755 ${DEST_DIR}/$@

clean:
	@rm -rf ${DEST_DIR}
