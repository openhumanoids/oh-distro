#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/loose_finger_cli /dev/ttyUSB0 burn ../../firmware/build/fmcb/std/fmcb-std.bin
