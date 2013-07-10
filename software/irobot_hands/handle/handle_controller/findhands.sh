#! /bin/bash

for d in {1..254}
do
    echo "trying 192.168.40.$d"
    ping 192.168.40.$d -c 1 > /dev/null
    
    if [[ $? == 0 ]]
    then
        echo "  Found hand at: 192.168.40.$d"
    fi
done
