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
echo "This will flash new firmware onto the palm microcontroller."
confirm "Do you want to continue? [y/N]:"
set +o errexit

ERRORS=""

BOARD="Palm"
echo ""
echo "Flashing $BOARD"
./bootload 0 && sleep 1 && ./firmware-up-palm palm.hex
if [ $? -eq 0 ]
then
    echo "$BOARD done." 
else
    echo "$BOARD ERROR." 
    ERRORS=$ERRORS"$BOARD failed to program\n"
fi

echo ""

if [ -z "$ERRORS" ]
then
    echo "Success"
else
    echo "Finished with errors:"
    echo -e $ERRORS
fi
