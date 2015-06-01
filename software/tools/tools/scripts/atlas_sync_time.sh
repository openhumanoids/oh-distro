#!/bin/bash

service ntp stop

if [ "atlas0" = $(hostname) || "paladin-12" = $(hostname) ]
then
    ntpdate 10.5.3.254
else
    ntpdate inter-atlas0
fi

service ntp start

echo synchronized clock on `hostname`
