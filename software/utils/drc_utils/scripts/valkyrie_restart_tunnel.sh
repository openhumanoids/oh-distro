#!/bin/bash

#set -x
source '/home/val/openhumanoids/oh-distro/software/config/drc_environment.sh'
if [[ "$1" == "server" ]]; then
    command="bot-lcm-tunnel link02"
else
    command="bot-lcm-tunnel"
fi


killall -qw bot-lcm-tunnel
screen -D -m -S tunnel $command &

until pids=$(pidof bot-lcm-tunnel); do sleep 0.1; done
echo started lcm tunnel on `hostname`: $command
