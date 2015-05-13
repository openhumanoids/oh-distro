#!/bin/bash

service ntp stop

if [ "atlas0" = $(hostname) ]
then
    ntpd -gq
else
    ntpdate inter-atlas0
fi

service ntp start

echo synchronized clock on `hostname`
