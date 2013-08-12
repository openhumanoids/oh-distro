#!/bin/bash
set -o verbose
set -o errexit
SANDIA_HAND=~/sandia-hand
FIRMWARE=$SANDIA_HAND/firmware
DRIVER=$SANDIA_HAND/ros/sandia_hand_driver
CLI=$DRIVER/bin/sandia_hand_cli
$CLI fp 0 on
sleep 5
$CLI pp 0 on
sleep 1
make -C $FIRMWARE/build f2-bl-gpnvm
sleep 1
make -C $FIRMWARE/build f2-bl-program
sleep 5
$CLI f2burn 0 $FIRMWARE/build/f2/std/f2-std.bin
sleep 2
$CLI pp 0 off
sleep 1
$CLI fp 0 off
