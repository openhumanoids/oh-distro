#!/bin/bash
chmod a+x ~/.bashrc
PS1='$ '
source ~/.bashrc
env
/home/drc/drc/software/build/bin/bot-procman-sheriff -l /home/drc/drc/software/config/drc_robot.pmd
