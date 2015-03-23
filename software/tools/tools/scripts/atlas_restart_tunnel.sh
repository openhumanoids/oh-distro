#!/bin/bash

set -x

if [[ "$1" == "server" ]]; then
    command="bot-lcm-tunnel"
else
    command="bot-lcm-tunnel 10.5.3.11 -l $LCM_URL_DRC_RADIO"
fi

pidof bot-lcm-tunnel && killall bot-lcm-tunnel
screen -D -m -S tunnel $command &
pidof bot-lcm-tunnel && echo started tunnel: `hostname`
