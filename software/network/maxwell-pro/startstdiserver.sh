#!/bin/bash

#**********************************************************************
# $Id: startstdiserver.sh 31 2015-02-02 23:11:51Z karl $
#**********************************************************************

STDILOG=/tmp/stdiserver_log.txt

/usr/local/bin/killstdiserver /dev/null 2>&1 || true

if [ -f ${STDILOG} ] ; then
  lgsz=`ls -l ${STDILOG} | awk '{print $5;}'`
  if expr ${lgsz} \> 1000000 > /dev/null 2>&1; then
    > ${STDILOG}
  chmod 0666 ${STDILOG} || true
  fi
fi

/usr/local/bin/stdiserver -t 2 -b 131072 -m 1500 -B 1536 -i 10 -h p1p2 -P 7021 -l p1p1 >> ${STDILOG} 2>&1 &
