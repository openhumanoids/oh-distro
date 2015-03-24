#!/bin/bash

pidof drc-lcm-bridge && killall drc-lcm-bridge
screen -D -m -S bridge drc-lcm-bridge -n `hostname` -c ${DRC_BASE}/software/config/drc_robot.cfg &
#daemon --command="drc-lcm-bridge -n `hostname` -c ${DRC_BASE}/software/config/drc_robot.cfg" > $HOME/bridge_output.txt 2>&1
pidof drc-lcm-bridge && echo started bridge: `hostname`
