#!/bin/bash

# cd to the log directory
cd /home/edbot/logs/video-logs || exit

# clean up old logs
rm lcmlog*

# start logger with automatic splitting
LCM_DEFAULT_URL=udpm://239.255.76.50:7650?ttl=0 lcm-logger --split-mb 1000
