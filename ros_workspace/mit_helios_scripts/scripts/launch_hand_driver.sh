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

if [[ -z "$2" ]] #if param 2 is empty
then
    echo " "
    echo "INPUT ERROR - second argument is missing"
    usage
    exit 1
fi


if [[ -z "$3" ]] #if param 3 is empty
then
   echo " " 
elif [ "$3" -ne "47" ] && [ "$3" -ne "49" ] 
then
    echo " "
    echo "INPUT ERROR - third argument must be 47 or 49"
    usage
    exit 1
fi


if [ "$1" == "IROBOT" ]
then
    if [ "$2" == "L" ]
    then

      if [[ -z "$3" ]] #if param 3 is empty
      then
          echo "launching irobot-hand driver for left hand @ 192.168.40.47 ..."
          roslaunch handle_launch bringup_left.launch left_hand_name:="192.168.40.47"
      else
          echo "launching irobot-hand driver for left hand @ 192.168.40.$3 ..."
          roslaunch handle_launch bringup_left.launch left_hand_name:="192.168.40.$3"
      fi

    elif [ "$2" == "R" ]
    then
      if [[ -z "$3" ]] #if param 3 is empty
      then
          echo "launching irobot-hand driver for right hand @ 192.168.40.49 ..."
          roslaunch handle_launch bringup_right.launch right_hand_name:="192.168.40.49"
      else
          echo "launching irobot-hand driver for right hand @ 192.168.40.$3 ..."
          roslaunch handle_launch bringup_right.launch right_hand_name:="192.168.40.$3"
      fi
    else
      echo " "
      echo "INPUT ERROR - unknown second argument..."
      usage
      exit 1
    fi
elif [ "$1" == "SANDIA" ]
then
    if [ "$2" == "L" ]
    then
      echo "launching sandia-hand driver for left hand @ 10.66.171.22 ..."
       rosrun sandia_hand_driver sandia_hand_node __ns:=sandia_hands/l_hand _ip:=10.66.171.22 _port:=12325     
    elif [ "$2" == "R" ]
    then
       echo "launching sandia-hand driver for right hand @ 10.66.171.23 ..."
       rosrun sandia_hand_driver sandia_hand_node __ns:=sandia_hands/r_hand _ip:=10.66.171.23 _port:=12321
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
