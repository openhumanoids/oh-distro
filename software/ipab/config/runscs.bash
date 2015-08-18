#!/bin/bash
source /usr/share/gazebo/setup.sh 
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/models/

cd ~/workspace/ValkyrieHardwareDrivers
java -jar lib/ValkyrieIPABSimulator-32172.jar
