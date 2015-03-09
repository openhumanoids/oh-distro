#!/bin/bash

#set -x

source /home/pat/source/drc/drc/software/config/drc_environment.sh

killall -q drc-network-shaper
killall -q bot-lcm-tunnel
killall -q bot-procman-deputy

sleep 1

export LCM_DEFAULT_URL=$LCM_URL_DRC_ROBOT 

screen -D -m -S shaper-link3 drc-network-shaper -r robot -c drc_robot.cfg -l -p /tmp  -i link3 -v &
screen -D -m -S shaper-link2 drc-network-shaper -r robot -c drc_robot.cfg -l -p /tmp  -i link2 -v &
screen -D -m -S deputy bot-procman-deputy -n field &
screen -D -m -S tunnel bot-lcm-tunnel & 

sleep 1
screen -ls
