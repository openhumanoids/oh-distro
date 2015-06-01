#!/bin/bash

service ntp stop
ntpd -gq
service ntp start

echo synchronized clock on `hostname`
