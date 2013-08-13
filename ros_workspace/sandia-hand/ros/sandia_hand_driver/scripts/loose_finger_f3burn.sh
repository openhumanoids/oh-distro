#!/bin/bash
set -o verbose
set -o errexit
cd `rospack find sandia_hand_driver` && bin/loose_finger_cli /dev/ttyUSB0 dburn ../../firmware/build/f3/std/f3-std.bin

