#!/bin/bash

if [ $# -ne 3 ]
then
    echo "Usage: `basename $0` rate_kbps delay_ms drop_%"
    exit -1
fi

I=lo
PORT=9001
RATE=$1kbps
DELAY=$2ms
DROP=$3%
echo Rate=$RATE, Delay=$DELAY, Drop=$DROP
sudo tc qdisc del dev $I root 2> /dev/null
sudo tc qdisc add dev $I handle 1: root htb
sudo tc class add dev $I parent 1: classid 1:1 htb rate $RATE
sudo tc qdisc add dev $I parent 1:1 handle 10: netem delay $DELAY loss $DROP
sudo tc filter add dev $I protocol ip parent 1: prio 1 u32 match \
  ip dport $PORT 0xffff flowid 1:1
sudo tc filter add dev $I protocol ip parent 1: prio 1 u32 match \
  ip sport $PORT 0xffff flowid 1:1



