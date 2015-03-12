#!/bin/bash

#set -x

# killall -q bot-lcm-tunnel
killall -q bot-procman-deputy
killall -q drc-lcm-bridge

sleep 1

LCM_DEFAULT_URL=$LCM_URL_DRC_PERCEPTION screen -D -m -S deputy-atlas0 bot-procman-deputy -n atlas0 &
LCM_DEFAULT_URL=$LCM_URL_DRC_PERCEPTION screen -D -m -S deputy-atlas1 bot-procman-deputy -n atlas1 &
LCM_DEFAULT_URL=$LCM_URL_DRC_CONTROL screen -D -m -S deputy-atlas2 bot-procman-deputy -n atlas2 &
#LCM_DEFAULT_URL=$LCM_URL_DRC_RADIO screen -D -m -S tunnel bot-lcm-tunnel 10.5.3.11 & 
LCM_DEFAULT_URL=$LCM_URL_DRC_RADIO screen -D -m -S tunnel bot-lcm-tunnel localhost -p 9999 & 
LCM_DEFAULT_URL=$LCM_URL_DRC_PERCEPTION screen -D -m -S bridge-atlas0 drc-lcm-bridge -n atlas0 -c ~/drc/software/config/drc_robot.cfg&
screen -D -m -S bridge-atlas1 drc-lcm-bridge -n atlas1 -c ~/drc/software/config/drc_robot.cfg&
LCM_DEFAULT_URL=$LCM_URL_DRC_CONTROL screen -D -m -S bridge-atlas2 drc-lcm-bridge -n atlas2 -c ~/drc/software/config/drc_robot.cfg&

sleep 1
screen -ls
