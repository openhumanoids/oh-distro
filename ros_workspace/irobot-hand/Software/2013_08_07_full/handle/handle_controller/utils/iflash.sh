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

REPLY=""
function deviceFile()
{
    case "$1" in
        "0")
            REPLY="palm.hex" ;;
        "1" | "3" | "5")
            REPLY="proximal.hex" ;;
        "2" | "4" | "6")
            REPLY="distal.hex" ;;
        "7" | "8" | "9" | "10")
            REPLY="motor.hex" ;;
        "11")
            REPLY="tactile.hex" ;;
        *)
            REPLY="" ;;
    esac
}

function deviceNum()
{
    # convert to lower case
    LOWERCASE=`echo "$1" | tr A-Z a-z`
    
    case "$LOWERCASE" in
        "palm" | "p" | "0")
            REPLY="0" ;;
        "proximals" | "r")
            REPLY="1 3 5" ;;
        "distals" | "d")
            REPLY="2 4 6" ;;
        "motors" | "m")
            REPLY="7 8 9 10" ;;
        "tactile" | "t" | "11")
            REPLY="11" ;;
        "fingers" | "f")
            REPLY="1 2 3 4 5 6" ;;
        "all" | "a")
            REPLY="0 1 2 3 4 5 6 7 8 9 10 11" ;;
        "finger1" | "f1")
            REPLY="3 4" ;;
        "finger2" | "f2")
            REPLY="1 2" ;;
        "finger3" | "f3")
            REPLY="5 6" ;;
        "motor1" | "m1" | "10")
            REPLY="10" ;;
        "motor2" | "m2" | "8")
            REPLY="8" ;;
        "motor3" | "m3" | "9")
            REPLY="9" ;;
        "motor4" | "m4" | "7")
            REPLY="7" ;;
        "finger1proximal" | "f1p" | "3")
            REPLY="3" ;;
        "finger1distal" | "f1d" | "4")
            REPLY="4" ;;
        "finger2proximal" | "f2p" | "1")
            REPLY="1" ;;
        "finger2distal" | "f2d" | "2")
            REPLY="2" ;;
        "finger3proximal" | "f3p" | "5")
            REPLY="5" ;;
        "finger3distal" | "f3d" | "6")
            REPLY="6" ;;
        *)
            REPLY="" ;;
    esac
}

function usage()
{
    echo "usage:"
    echo "  iflash.sh [micro1] [micro2] ..."
    echo "You can specify each micro like so:"
    echo "  palm, proximals, distals, motors, tactile, fingers, all"
    echo "  p, r, d, m, t, f, a"
    echo "  finger1, finger2, finger3"
    echo "  f1, f2, f3"
    echo "  motor1, motor2, motor3, motor4"
    echo "  m1, m2, m3, m4"
    echo "  f1p, f1d, f2p, f2d, f3p, f3d"
    echo "  finger1proximal, finger1distal"
    echo "  finger2proximal, finger2distal"
    echo "  finger3proximal, finger3distal"
    echo "  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11"
}

if [[ -z "$1" ]] #if param 1 is empty
then
    echo "ERROR: must pass in which micro(s) to flash"
    usage
    exit 1
fi

ERRORS=""
DEVICES=""

# Parse command line arguments into device numbers
for DEVSTRING in $@
do
    deviceNum $DEVSTRING
    DEVICES="$DEVICES $REPLY"
done

# 
# FILES=""
# for DEV in $DEVICES
# do
#     deviceFile $DEV
#     FILES="$FILES $REPLY"
# done

# convert to array
DEVICEARRAY=( $DEVICES )

set -o errexit
echo ""
echo "WARNING!"
if [ ${#DEVICEARRAY[@]} == "0" ]
then
    echo "ERROR: No valid devices"
    usage
    set +o errexit
    exit 1
elif [ ${#DEVICEARRAY[@]} == "1" ]
then
    echo "This will flash new firmware onto this microcontroller:"
else
    echo "This will flash new firmware onto these microcontrollers:"
fi
echo $DEVICES
confirm "Do you want to continue? [y/N]:"
set +o errexit


for DEVICE in $DEVICES
do
    deviceFile $DEVICE
    echo ""
    echo "Flashing $DEVICE with $REPLY"
    if [ "$DEVICE" == "0" ]
    then
        ./bootload $DEVICE && sleep 1 && ./firmware-up-palm $REPLY
    else
        ./bootload $DEVICE && sleep 1 && ./firmware-up-not-palm $REPLY
    fi
    
    if [ $? -eq 0 ]
    then
        echo "$DEVICE done, waiting for restart." 
    else
        echo "$DEVICE ERROR, waiting for restart." 
        ERRORS=$ERRORS"$DEVICE failed to program\n"
    fi
    sleep 35
done

echo ""

if [ -z "$ERRORS" ]
then
    echo "Success"
else
    echo "Failed with errors:"
    echo -e $ERRORS
fi
