#! /bin/bash

rosnode kill /annan_control 

FINGER_COUNT=0
SPREAD_COUNT=0
while :
do
    echo "Finger Count: "$FINGER_COUNT"  Spread Count: "$SPREAD_COUNT
    
    if [[ ! -z "$1" ]] #if param 1 is not empty
    then
        if [[ $FINGER_COUNT -gt $1 ]]
        then
            echo "Done"
            break
        fi
    fi
    
    for i in {1..4}
    do
        let FINGER_COUNT=FINGER_COUNT+1
        
        rostopic pub -1 /right_hand/control handle_msgs/HandleControl '{type: [2, 2, 2, 0, 0], value: [5000, 5000, 5000, 0, 0], valid: [True, True, True, False, False]}' 
        rostopic pub -1 /right_hand/control handle_msgs/HandleControl '{type: [2, 2, 2, 0, 0], value: [0, 0, 0, 0, 0], valid: [True, True, True, False, False]}' 
        echo "Finger Count: "$FINGER_COUNT"  Spread Count: "$SPREAD_COUNT
    done
    
    let SPREAD_COUNT=SPREAD_COUNT+1
    
    rostopic pub -1 /right_hand/control handle_msgs/HandleControl '{type: [0, 0, 2, 2, 2], value: [0, 0, 5000, 2000, 768], valid: [False, False, True, True, True]}' 
    rostopic pub -1 /right_hand/control handle_msgs/HandleControl '{type: [0, 0, 2, 2, 2], value: [0, 0, 0, 0, 0], valid: [False, False, True, True, True]}' 

done
