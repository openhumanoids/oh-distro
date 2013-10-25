#! /bin/bash

function confirm()
{
    echo -n "$@ "
    read -e answer
    for response in y Y yes YES Yes
    do
        if [ "_$answer" == "_$response" ]
        then
            return 0
        fi
    done

    # Any answer other than the list above is considerred a "no" answer
    return 1
}

set -o errexit
echo ""
echo "WARNING!"
echo "This will flash new firmware onto each of the finger microcontrollers."
confirm "Do you want to continue? [y/N]:"
set +o errexit

ERRORS=""

for NUM in 1 3 5
do
    BOARD="Device "$NUM
    echo ""
    echo "Flashing $BOARD"
    ./bootload $NUM && sleep 1 && ./firmware-up-not-palm proximal.hex
    if [ $? -eq 0 ]
    then
        echo "$BOARD done, waiting for restart." 
    else
        echo "$BOARD ERROR, waiting for restart." 
        ERRORS=$ERRORS"$BOARD failed to program\n"
    fi
    sleep 35
done

for NUM in 2 4 6
do
    BOARD="Device "$NUM
    echo ""
    echo "Flashing $BOARD"
    ./bootload $NUM && sleep 1 && ./firmware-up-not-palm distal.hex
    if [ $? -eq 0 ]
    then
        echo "$BOARD done, waiting for restart." 
    else
        echo "$BOARD ERROR, waiting for restart." 
        ERRORS=$ERRORS"$BOARD failed to program\n"
    fi
    sleep 35
done

echo ""

if [ -z "$ERRORS" ]
then
    echo "Success"
else
    echo "Finished with errors:"
    echo -e $ERRORS
fi
