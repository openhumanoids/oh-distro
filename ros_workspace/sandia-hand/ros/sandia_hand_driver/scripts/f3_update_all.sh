#!/bin/bash
set -o errexit
set -o verbose
SANDIA_HAND=`rospack find sandia_hand_driver`/../..
FIRMWARE=$SANDIA_HAND/firmware
DRIVER=$SANDIA_HAND/ros/sandia_hand_driver
CLI=$DRIVER/bin/sandia_hand_cli
FINGERS="0" # 1 2 3"
echo "powering up motor modules..."
for FINGER_IDX in $FINGERS; do
  $CLI fp $FINGER_IDX low
done
sleep 5
echo "ramping up power to all sockets..."
for FINGER_IDX in $FINGERS; do
  $CLI fp $FINGER_IDX on 
done
sleep 1
echo "programming distal phalanges..."
for FINGER_IDX in $FINGERS; do
  $CLI f3burn $FINGER_IDX $FIRMWARE/build/f3/std/f3-std.bin
done
echo "powering down motor modules..."
for FINGER_IDX in $FINGERS; do
  $CLI fp $FINGER_IDX off
done
echo "done!"
