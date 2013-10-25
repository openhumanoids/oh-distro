#! /bin/bash

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
    
    ./motortest 5 2 0 1000
    sleep 4
    ./motortest 5 2 0 -100
    sleep 4

done
