#!/bin/bash

#set -x

if [[ "$1" == "server" ]]; then
    command="bot-lcm-tunnel"
else
    command="bot-lcm-tunnel 10.5.3.11 -l $LCM_URL_DRC_RADIO"
fi

killall -qw bot-lcm-tunnel
screen -D -m -S tunnel $command &

until pids=$(pidof bot-lcm-tunnel); do sleep 0.1; done
echo started lcm tunnel on `hostname`: $command
