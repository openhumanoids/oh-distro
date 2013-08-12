
objects := motortest \
	parametertest \
	parametertest2 \
	sensor_poll \
	sensor_spin \
	stopall \
	bootload \
	raw_spin \
	calibrate_opcode \
	debug_poll \
	ledflash \
	test_palm_tactile \
	test_motor \
	chainmask \
	get_sensor

scripts := killhandle.sh \
	calibrate_all.sh \
	flash_fingers.sh \
	flash_distals.sh \
	flash_proximals.sh \
	flash_motors.sh \
	flash_palm.sh \
	flash_palm_firsttime.sh \
	flash_tactile.sh \
	flash_all.sh \
	firmware-up-not-palm \
	firmware-up-palm \
	get_firmware_versions.sh \
	testscript.sh \
	breakinspread.sh \
	iflash.sh

all: $(objects)

% : %.c
	arm-commonos-linux-gnueabi-g++ $< ../../handle_lib/src/packing.cpp -Wall -pthread -ggdb3 -o $@

clean:
	rm *~ $(objects)

install:
	scp $(objects) $(scripts) root@armH-palm-1:
