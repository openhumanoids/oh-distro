#!/bin/bash

loopback_interface=`ifconfig | grep -m 1 -i loopback | cut -d : -f1 | cut -d ' ' -f1`;
#echo loopback interface is $loopback_interface

if [ `uname -s` == "Darwin" ]; then
  # loopback appears to be multicast by default...
  sudo route add -net 224.0.0.0 -netmask 240.0.0.0 -interface $loopback_interface 
else
  sudo ifconfig $loopback_interface multicast;
  sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
fi
