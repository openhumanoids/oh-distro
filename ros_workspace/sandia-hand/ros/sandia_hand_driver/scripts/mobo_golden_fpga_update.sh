#!/bin/bash
cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli $1 mflash_burn_golden_fpga mobo.bin
