#!/bin/bash

killall -qw bot-procman-deputy
daemon --command="bot-procman-deputy -n `hostname`" > $HOME/deputy_output.txt 2>&1

until pids=$(pidof bot-procman-deputy); do sleep 0.1; done
echo started deputy on `hostname` with LCM url: $LCM_DEFAULT_URL
