#!/bin/bash

killall -qw drc-lcm-bridge
screen -D -m -S bridge drc-lcm-bridge -n `hostname` -c ${DRC_BASE}/software/config/atlas/robot.cfg &

until pids=$(pidof drc-lcm-bridge); do sleep 0.1; done
echo started bridge on `hostname`
