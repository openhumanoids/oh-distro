#!/bin/bash

ifconfig lo multicast

for url in $LCM_URL_DRC_CONTROL $LCM_URL_DRC_PERCEPTION $LCM_URL_DRC_ATLAS_0_2 $LCM_URL_DRC_ATLAS_1_2 $LCM_URL_DRC_RADIO $LCM_URL_DRC_DEFAULT $LCM_URL_DRC_ROBOT $LCM_URL_DRC_BASE; 
do 
ip=`echo $url | sed 's|udpm://||' | sed 's|:[0-9]\+?ttl=[0-9]||'`
echo $ip
route add $ip dev lo
done

