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
echo "This will flash new firmware onto the tactile microcontroller."
confirm "Do you want to continue? [y/N]:"
set +o errexit

ERRORS=""

BOARD="Tactile"
echo ""
echo "Flashing $BOARD"
./bootload 11 && sleep 1 && ./firmware-up-not-palm tactile.hex
if [ $? -eq 0 ]
then
    echo "$BOARD done, waiting for restart." 
else
    echo "$BOARD ERROR, waiting for restart." 
    ERRORS=$ERRORS"$BOARD failed to program\n"
fi
sleep 35

echo ""

if [ -z "$ERRORS" ]
then
    echo "Success"
else
    echo "Finished with errors:"
    echo -e $ERRORS
fi
