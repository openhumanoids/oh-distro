#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 0 off
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 1 off
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 2 off
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 3 off
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 mmburn_all ../../firmware/build/fmcb/std/fmcb-std.bin
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 0 off
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 1 off
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 2 off
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 fp 3 off
