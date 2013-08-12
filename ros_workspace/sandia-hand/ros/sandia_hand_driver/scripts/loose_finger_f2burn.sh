#!/bin/bash
set -o verbose
set -o errexit
cd `rospack find sandia_hand_driver` && bin/loose_finger_cli /dev/ttyUSB0 pburn ../../firmware/build/f2/std/f2-std.bin

