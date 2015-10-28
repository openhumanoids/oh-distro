#!/bin/bash
JAR_FILE=lib/ValkyrieIPABSimulator-32172.jar

if [ ! -f "/usr/share/gazebo/setup.sh" ]; then
	echo "ERROR: Can't find /usr/share/gazebo/setup.sh"
	echo "Have you installed Gazebo?"
	exit 1
fi

source /usr/share/gazebo/setup.sh 
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/models/

if [ ! -d "$HOME/workspace/ValkyrieHardwareDrivers" ]; then
	echo "ERROR: Can't find ~/workspace/ValkyrieHardwareDrivers"
	echo "Have you checked out the IHMC repository?"
	exit 1
fi

cd ~/workspace/ValkyrieHardwareDrivers

if [ ! -f "$JAR_FILE" ]; then
	echo "ERROR: Can't find $JAR_FILE"
	echo "Have you compiled the correct version of JAR file?"
	exit 1
fi

java -jar $JAR_FILE
