#!/bin/sh
# Sets up the environment and starts a deputy to make sure
# we can control the Kuka remotely

# Exclusively for WENDY

# This script has been tested with Wolfgang's user account 
# but will work with any other as long as LCM is installed 
# and bot-procman-deputy is in the home directory
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
export LCM_DEFAULT_URL=udpm://239.255.76.67:1337?ttl=1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/`whoami`/local/lib # this is needed as without sudo, LCM is installed locally
export PATH=$PATH:/home/`whoami`/local/bin

./bot-procman-deputy -n "wendy"