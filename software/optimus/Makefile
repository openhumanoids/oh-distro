GPU_CMD="vblank_mode=0 primusrun"
#GPU_CMD="optirun"

DRC_ROOT=${HOME}/drc
DRC_BIN=${DRC_ROOT}/software/build/bin
DEST_DIR = ${DRC_ROOT}/software/build/optimus

FULL_BINS=\
${DRC_BIN}/drc-viewer\
${DRC_BIN}/drc-image-passthrough\
${DRC_BIN}/drc-lidar-passthrough\
${DRC_BIN}/stereo-bm\
/usr/bin/gzserver\
/usr/bin/gzclient

BARE_BINS=$(notdir ${FULL_BINS})
SHELL_CMDS=$(addprefix ${DEST_DIR}/, ${BARE_BINS})

ARGS_STR=\"$$\{@:1\}\"

.PHONY: ${FULL_BINS}

default: clean dir ${FULL_BINS}

dir:
	@mkdir -p ${DEST_DIR}
	@ln -fs ${DRC_BIN}/bot-procman-deputy ${DEST_DIR}/

${FULL_BINS}:
	@echo "Creating $@"
	@echo "#!/bin/bash" > ${DEST_DIR}/$(notdir $@)
	@echo "${GPU_CMD} $@" ${ARGS_STR} >> ${DEST_DIR}/$(notdir $@)
	@chmod 755 ${DEST_DIR}/$(notdir $@)

clean:
	rm -rf ${DEST_DIR}
