#! /bin/bash

if [[ -z "$1" ]] #if param 1 is empty
then
    echo "ERROR: Must pass in hand number"
    exit 1
fi

let "HAND=$1+100"

scp proximal/proximal.hex distal/distal.hex tactile/tactile.hex motor/motor.hex palm/palm.hex root@10.66.171.$HAND:

#root@192.168.40.$1:
