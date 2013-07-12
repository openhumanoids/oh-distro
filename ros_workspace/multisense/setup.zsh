#!/usr/bin/env zsh
# THIS IS AN AUTO-GENERATED FILE
# IT IS UNLIKELY YOU WANT TO EDIT THIS FILE BY HAND
# IF YOU WANT TO CHANGE THE ROS ENVIRONMENT VARIABLES
# USE THE rosinstall OR rosws TOOL INSTEAD.
# Generator version: 0.6.29
# see: http://www.ros.org/wiki/rosinstall


CATKIN_SHELL=zsh

SCRIPT_PATH="$(dirname $0)"

# Load the path of this particular setup.zsh

if [ ! -f "$SCRIPT_PATH/setup.sh" ]; then
  echo "Bug: shell script unable to determine its own location: $SCRIPT_PATH"
  return 22
fi

# unset _ros_decode_path (function of rosbash) to check later whether setup.sh has sourced roszsh
unset -f _ros_decode_path 1> /dev/null 2>&1


emulate sh # emulate POSIX
. $SCRIPT_PATH/setup.sh
emulate zsh # back in zsh


# if we have a ROS_ROOT, then we might need to source rosbash (pre-fuerte)
if [ ! -z "${ROS_ROOT}" ]; then
  # check whether setup.sh also already sourced rosbash
  # Cannot rely on $? due to set -o errexit in build scripts
  RETURNCODE=`type _ros_decode_path 2> /dev/null | grep function 1>/dev/null 2>&1 || echo error`

  # for ROS electric and before, source rosbash
  if [ ! "$RETURNCODE" = "" ]; then
    RETURNCODE=`rospack help 1> /dev/null 2>&1 || echo error`
    if  [ "$RETURNCODE" = "" ]; then
      ROSSHELL_PATH=`rospack find rosbash`/roszsh
      if [ -e "$ROSSHELL_PATH" ]; then
        . $ROSSHELL_PATH
      fi
    else
      echo "rospack could not be found, you cannot have roszsh features until you bootstrap ros"
    fi
  fi
fi
