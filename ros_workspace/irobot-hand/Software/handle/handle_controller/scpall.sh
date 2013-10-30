#! /bin/bash

if [[ -z "$1" ]] #if param 1 is empty
then
    echo "ERROR: Must pass in hand number"
    exit 1
fi

let "HAND=$1+100"

scp src/handle_controller utils/motortest utils/parametertest utils/parametertest2 utils/sensor_poll utils/sensor_spin utils/stopall utils/bootload utils/raw_spin utils/calibrate_opcode utils/debug_poll utils/ledflash utils/killhandle.sh utils/calibrate_all.sh utils/flash_fingers.sh utils/flash_distals.sh utils/flash_proximals.sh utils/flash_motors.sh utils/flash_palm.sh utils/flash_palm_firsttime.sh utils/flash_tactile.sh utils/flash_all.sh utils/firmware-up-not-palm utils/firmware-up-palm utils/get_firmware_versions.sh utils/testscript.sh utils/breakinspread.sh utils/test_palm_tactile utils/test_motor utils/chainmask utils/get_sensor utils/iflash.sh utils/resetmotorparams.sh root@10.66.171.$HAND:

#root@192.168.40.$1:
