#!/bin/bash

pidof bot-procman-deputy && killall bot-procman-deputy
LCM_DEFAULT_URL=udpm://239.255.76.68:7668?ttl=1 bot-procman-deputy -n `hostname` > $HOME/deputy_output.txt 2>&1  &
sleep 0.1
pidof bot-procman-deputy && echo started deputy: `hostname`

