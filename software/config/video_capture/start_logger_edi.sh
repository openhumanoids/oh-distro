#!/bin/bash

# Check whether workstation is configured to be used as a video capture station
if [ -z "$OH_VIDEO_CAPTURE" || -z "$OH_VIDEO_CAPTURE_LOCATION" ]; then
    echo "Need to set OH_VIDEO_CAPTURE=1 and OH_VIDEO_CAPTURE_LOCATION after configuring as video capture station"
    exit 1
fi

# Check that logging directory exists, else create it
if [ ! -d "$OH_VIDEO_CAPTURE_LOCATION" ]; then
   mkdir -p "$OH_VIDEO_CAPTURE_LOCATION"
fi

# cd to the log directory
cd "$OH_VIDEO_CAPTURE_LOCATION" || exit

# clean up old logs
#rm lcmlog*

# start logger with automatic splitting
LCM_DEFAULT_URL=udpm://239.255.76.50:7650?ttl=0 lcm-logger --split-mb 1000
