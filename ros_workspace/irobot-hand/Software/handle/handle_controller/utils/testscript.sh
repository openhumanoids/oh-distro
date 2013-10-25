#! /bin/bash

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
        
        ./motortest 4 2 0 5000
        ./motortest 2 2 0 5000
        ./motortest 3 2 0 5000
        sleep 2
        ./motortest 4 2 0 0
        ./motortest 2 2 0 0
        ./motortest 3 2 0 0
        sleep 2
        
        echo "Finger Count: "$FINGER_COUNT"  Spread Count: "$SPREAD_COUNT
    done
    
    let SPREAD_COUNT=SPREAD_COUNT+1
    
    ./motortest 3 2 0 5000
    ./motortest 1 2 0 2000
    ./motortest 5 2 0 768
    sleep 3
    ./motortest 3 2 0 0
    ./motortest 1 2 0 0
    ./motortest 5 2 0 0
    sleep 3

done
