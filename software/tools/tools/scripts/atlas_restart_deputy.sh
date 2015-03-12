#!/bin/bash

pidof bot-procman-deputy && killall bot-procman-deputy
daemon --command="bot-procman-deputy -n `hostname`" > $HOME/deputy_output.txt 2>&1
sleep 0.1
pidof bot-procman-deputy && echo started deputy: `hostname` with LCM url: $LCM_DEFAULT_URL
