#!/bin/bash

source '/home/val/openhumanoids/oh-distro/software/config/drc_environment.sh'
killall -qw bot-procman-deputy
if [[ "$HOSTNAME" == "vis02"  ]]; then
	daemon --command="bot-procman-deputy -n base" > $HOME/deputy_output.txt 2>&1
else
	daemon --command="bot-procman-deputy -n `hostname`" > $HOME/deputy_output.txt 2>&1
fi

# daemon --command="bot-procman-deputy -n  base" > $HOME/deputy_output.txt 2>&1


until pids=$(pidof bot-procman-deputy); do sleep 0.1; done
echo started deputy on `hostname` with LCM url: $LCM_DEFAULT_URL
