#!/bin/bash
source /usr/share/gazebo/setup.sh 
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/models/

cd ~/workspace_alpha/ValkyrieHardwareDrivers
java -jar lib/ValkyrieIPABSimulator-32468.jar
