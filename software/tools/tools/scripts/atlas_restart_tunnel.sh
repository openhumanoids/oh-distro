#!/bin/bash

pidof bot-lcm-tunnel && killall bot-lcm-tunnel
screen -D -m -S tunnel bot-lcm-tunnel 10.5.3.11 &
pidof bot-lcm-tunnel && echo started tunnel: `hostname`
