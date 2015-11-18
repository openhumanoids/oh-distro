#!/bin/bash

#service ntp stop

#if [ "atlas0" = $(hostname) ]
#then
#    ntpdate 10.5.3.254
#elif [ "paladin-12" = $(hostname) ]
#then
#    ntpdate 10.5.3.254
#else
#    ntpdate inter-atlas0
#fi

#service ntp start

ntpdate -u 0.ubuntu.pool.ntp.org
echo synchronized clock on `hostname`
