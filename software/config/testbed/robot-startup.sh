#!/bin/bash

#set -x

killall -q bot-lcm-tunnel
killall -q bot-procman-deputy
killall -q drc-lcm-bridge

sleep 1

screen -D -m -S deputy-atlas0 bot-procman-deputy -n atlas0 &
screen -D -m -S deputy-atlas1 bot-procman-deputy -n atlas1 &
screen -D -m -S deputy-atlas2 bot-procman-deputy -n atlas2 &
screen -D -m -S tunnel bot-lcm-tunnel 10.5.3.3& 
screen -D -m -S bridge-atlas0 drc-lcm-bridge -n atlas0 &
screen -D -m -S bridge-atlas1 drc-lcm-bridge -n atlas1 &
screen -D -m -S bridge-atlas2 drc-lcm-bridge -n atlas2 &

sleep 1
screen -ls
