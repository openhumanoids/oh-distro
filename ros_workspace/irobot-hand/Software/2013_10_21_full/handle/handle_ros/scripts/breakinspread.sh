#! /bin/bash

rosnode kill /annan_control 

SPREAD_COUNT=0
while :
do
    echo "Spread Count: "$SPREAD_COUNT
    
    if [[ ! -z "$1" ]] #if param 1 is not empty
    then
        if [[ $SPREAD_COUNT -gt $1 ]]
        then
            echo "Done"
            break
        fi
    fi
    
    let SPREAD_COUNT=SPREAD_COUNT+1
    
    rostopic pub -1 /right_hand/control handle_msgs/HandleControl '{type: [0, 0, 0, 0, 2], value: [0, 0, 0, 0, 1000], valid: [False, False, False, False, True]}'
    sleep 2
    rostopic pub -1 /right_hand/control handle_msgs/HandleControl '{type: [0, 0, 0, 0, 2], value: [0, 0, 0, 0, -100], valid: [False, False, False, False, True]}'
    sleep 2
done

rostopic pub -1 /right_hand/control handle_msgs/HandleControl '{type: [0, 0, 0, 0, 2], value: [0, 0, 0, 0, 0], valid: [False, False, False, False, True]}'
