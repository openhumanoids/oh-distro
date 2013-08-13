#!/bin/bash
set -o verbose
set -o errexit
SANDIA_HAND=~/sandia-hand
FIRMWARE=$SANDIA_HAND/firmware
DRIVER=$SANDIA_HAND/ros/sandia_hand_driver
CLI=$DRIVER/bin/sandia_hand_cli
$CLI fp 0 low
sleep 1
make -C $FIRMWARE/build fmcb-bl-gpnvm
sleep 1
make -C $FIRMWARE/build fmcb-bl-program
sleep 5
$CLI mmburn 0 $FIRMWARE/build/fmcb/std/fmcb-std.bin
