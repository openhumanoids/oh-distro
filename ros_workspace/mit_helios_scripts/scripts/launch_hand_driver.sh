#! /bin/bash

# Wrapper script for launching sandia or irobot hand drivers for the BDI Atlas Robot named Helios.
# Helios is packaged with specific iRobot hands with ids 47 and 49


usage(){
    echo "-------------------- "
    echo "USAGE: launch_hand_driver.sh <hand_type> <side> [<irobot-hand#>]"
    echo " "
    echo "Where "
    echo "  <hand_type> is: IROBOT or SANDIA"
    echo "  <side> is: L or R"
    echo "  <irobot-hand#> is: 192.168.40.<irobot-hand#>"
    echo "  <irobot-hand#> is optional (default 192.168.40.49 - Right, 192.168.40.47 - Left)"
    echo " "
    echo "-------------------- "
}

if [[ -z "$1" ]] #if param 1 is empty
then
    echo " "
    echo "INPUT ERROR - first argument is missing"
    usage
    exit 1
fi
hand_type=$1

if [[ -z "$2" ]] #if param 2 is empty
then
    echo " "
    echo "INPUT ERROR - second argument is missing"
    usage
    exit 1
fi
side=$2


if [ -z "$3"] && [ "$hand_type" = "IROBOT" ] #if param 3 is empty
then
   echo " "
   echo "INPUT ERROR - third argument is missing"
   usage
   exit 1
fi
irobot_hand_id=$3

if [ "$hand_type" == "IROBOT" ]
then
    irobot_base_ip="192.168.40."
    irobot_ip="$irobot_base_ip$irobot_hand_id"
    if [ "$side" == "L" ]
    then
      echo "launching irobot-hand driver for left hand @ $irobot_ip ..."
      roslaunch handle_launch bringup_left.launch left_hand_name:="$irobot_ip"
    elif [ "$side" == "R" ]
    then
      echo "launching irobot-hand driver for right hand @ $irobot_ip"
      roslaunch handle_launch bringup_right.launch right_hand_name:="$irobot_ip"
    else
      echo " "
      echo "INPUT ERROR - unknown second argument..."
      usage
      exit 1
    fi
elif [ "$hand_type" == "SANDIA" ]
then
    if [ "$side" == "L" ]
    then
      echo "launching sandia-hand driver for left hand @ 10.66.171.22 ..."
       rosrun sandia_hand_driver sandia_hand_node __ns:=sandia_hands/l_hand _ip:=10.66.171.22 _port:=12325 _use_cameras:=false     
    elif [ "$side" == "R" ]
    then
       echo "launching sandia-hand driver for right hand @ 10.66.171.23 ..."
       rosrun sandia_hand_driver sandia_hand_node __ns:=sandia_hands/r_hand _ip:=10.66.171.23 _port:=12321 _use_cameras:=false
    else
      echo " "
      echo "INPUT ERROR - unknown second argument..."
      usage
      exit 1
    fi
else
  echo " "
  echo "INPUT ERROR - unknown first argument..."
  usage
  exit 1
fi
