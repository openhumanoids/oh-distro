#!/usr/bin/env bash
# script to set 
# need to run as sudo
# mfallon feb 2013

sudo ifconfig eth0 10.10.72.56
sudo echo 16777215 > /proc/sys/net/core/rmem_max
sudo echo 16777215 > /proc/sys/net/core/wmem_max
sudo ifconfig eth0 mtu 7200
