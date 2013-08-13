#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/loose_palm_cli /dev/ttyUSB0 burn ../../firmware/build/palm/std/palm-std.bin
