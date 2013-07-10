#! /bin/bash

if [[ -z "$1" ]] #if param 1 is empty
then
    echo "INPUT ERROR"
    echo "Usage: control.sh <hand1#> [<hand2#>]"
    echo "Where <hand#> is: 192.168.40.<hand#>"
    echo "The second hand is optional"
    exit 1
fi

if [[ -z "$2" ]] #if param 2 is empty
then
    roslaunch handle_launch key_control_righthand.launch right_hand_name:="192.168.40.$1"
else
    roslaunch handle_launch key_control_twohands.launch right_hand_name:="192.168.40.$1" left_hand_name:="192.168.40.$2"
fi
