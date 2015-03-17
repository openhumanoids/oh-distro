#!/bin/bash

#**********************************************************************
# $Id: starttrack.sh 73 2015-03-11 20:55:17Z karl $
#**********************************************************************

#****************************************
# Say where to find the auto-start schedule
#****************************************
# Fetch the schedule from a web server
#SCHEDURI=http://iwl.com/drc2015/schedule.yaml

# Fetch the schedule locally
SCHEDURI=file:schedule.yaml

# Name the blackout CSV file
#--#BKOUTFILE=2015-2-2_Degraded_Comms_Schedule_Example.csv
BKOUTFILE=badComms-Mar06.csv

# This can be commented out to make everything run in manual mode
SCHEDPARM="-a ${SCHEDURI} "

PIDDIR=/tmp

DRCCTRL=drcctrl.py

SLOW_RATE=9600
ICMP_RATE=1024

#****************************************
# Provide the DCE IPv4 addresses
#****************************************
DUMMY_DCE_IP=""

TEST_DCE_IP=192.168.10.100
TEST_DCE_IP=${DUMMY_DCE_IP}
BLUE_DCE_IP=10.254.1.101
RED_DCE_IP=10.254.1.102
GREEN_DCE_IP=10.254.1.103
YELLOW_DCE_IP=10.254.1.104
SPARE_DCE_IP=10.254.1.105

#BLUE_DCE_IP=${DUMMY_DCE_IP}
#RED_DCE_IP=${DUMMY_DCE_IP}
#GREEN_DCE_IP=${DUMMY_DCE_IP}
#YELLOW_DCE_IP=${DUMMY_DCE_IP}
#SPARE_DCE_IP=${DUMMY_DCE_IP}

#RED_DCE_IP=192.168.1.20
#GREEN_DCE_IP=192.168.1.21
#BLUE_DCE_IP=192.168.1.22
#YELLOW_DCE_IP=192.168.1.23

#****************************************
# Track Names and colors
#****************************************
TEST_TRACK_NM="TEST Track"
TEST_TRACK_CLR="magenta"

RED_TRACK_NM="Red Track"
RED_TRACK_CLR="coral"

GREEN_TRACK_NM="Green Track"
GREEN_TRACK_CLR="pale green"

BLUE_TRACK_NM="Blue Track"
BLUE_TRACK_CLR="light blue"

YELLOW_TRACK_NM="Yellow Track"
YELLOW_TRACK_CLR="goldenrod"

SPARE_TRACK_NM="Spare Track"
SPARE_TRACK_CLR="magenta"

#****************************************
# Window positioning
#
# The application windows are 474 wide by 220 high
#
# X moves window to right/left
# Y moves window to up/down
#****************************************
APPX=474
APPY=220

XMARGIN=20
YMARGIN=20

XDELTA=540
YDELTA=300

TEST_X=${XMARGIN}
TEST_Y=${YMARGIN}

BLUE_X=${XMARGIN}
BLUE_Y=${YMARGIN}

GREEN_X=`expr ${BLUE_X} + ${XDELTA}`
GREEN_Y=${BLUE_Y}

RED_X=${BLUE_X}
RED_Y=`expr ${BLUE_Y} + ${YDELTA}`

YELLOW_X=${GREEN_X}
YELLOW_Y=${RED_Y}

SPARE_X=${RED_X}
SPARE_Y=`expr ${RED_Y} + ${YDELTA}`

# Comment-out the next four lines to let the window manager do the layout.
TEST_P="${TEST_X}x${TEST_Y}"
RED_P="${RED_X}x${RED_Y}"
GREEN_P="${GREEN_X}x${GREEN_Y}"
BLUE_P="${BLUE_X}x${BLUE_Y}"
YELLOW_P="${YELLOW_X}x${YELLOW_Y}"
SPARE_P="${SPARE_X}x${SPARE_Y}"

# Manual mode will be set to "yes" if requsted
MANUAL_MODE="no"

RETVAL=0

cleanup_and_exit() {
  exit $1
}

trap "cleanup_and_exit 1" SIGQUIT SIGINT

# Returns OK (0) if $1 contains $2.
# Otherwise returns Not OK (1).
strstr() {
  [ "${1#*$2*}" = "$1" ] && return 1
  return 0
}

# Parameter - the proposed IP address
check_dummy() {
if [ -z "$1" ] ; then
 echo "********************************"
 echo "*  WARNING - DUMMY IP ADDRESS  *"
 echo "********************************"
 echo ""
 echo -n " Proceed? (Y)es/(N)o? [N] "
 read answer
 if strstr "nN" "$answer" || [ "$answer" = "" ] ; then
   echo "ABORTING."
   cleanup_and_exit 1
  fi
fi
}

#****************************************
# Generic application launcher
#****************************************
# Parameters:
#  1 - Track name
#  2 - Track color
#  3 - Position
#  4 - Blackoutfile
#  5 - DCE IPv4
#  6 - PIDfile
#  7 - Slow link rate
#  8 - ICMP link rate
#  9 - Manual mode (either "yes" or "no")
launch_app() {
  check_dummy "$5"
  if [ -f $6 ] ; then
    pinnum=`cat $6`
    if [ -d /proc/${pinnum} ] ; then
      echo "********************************"
      echo "*  $1 is already running."
      echo "*  Aborting."
      echo "********************************"
      return
    fi
    rm -f $6
  fi
  if [ ! -z "$3" ] ; then
    posparm="-P $3"
  else
    posparm=""
  fi
  if [ "${9}" != "yes" ] ; then
    echo python ${DRCCTRL} ${SCHEDPARM} --trackname "$1" --syslog --trackcolor "$2" ${posparm} -b ${4} --slowrate "${7}" --icmprate "${8}" "$5"
    python ${DRCCTRL} ${SCHEDPARM} --trackname "$1" --syslog --trackcolor "$2" ${posparm} -b ${4} --slowrate "${7}" --icmprate "${8}" "$5" &
  else
    echo python ${DRCCTRL} --trackname "$1" --syslog --trackcolor "$2" ${posparm} -b ${4} --slowrate "${7}" --icmprate "${8}" "$5"
    python ${DRCCTRL} --trackname "$1" --syslog --trackcolor "$2" ${posparm} -b ${4} --slowrate "${7}" --icmprate "${8}" "$5" &
  fi
  PID=$!
  echo "${PID}" > ${6}
}

#****************************************
# Functions to launch the applications
#****************************************
test_app() {
  launch_app "${TEST_TRACK_NM}" "${TEST_TRACK_CLR}" "${TEST_P}" "${BKOUTFILE}" "${TEST_DCE_IP}" "${PIDDIR}/testtrack.pid" "${SLOW_RATE}" "${ICMP_RATE}" "${MANUAL_MODE}"
}

red_app() {
  launch_app "${RED_TRACK_NM}" "${RED_TRACK_CLR}" "${RED_P}" "${BKOUTFILE}" "${RED_DCE_IP}" "${PIDDIR}/redtrack.pid" "${SLOW_RATE}" "${ICMP_RATE}" "${MANUAL_MODE}"
}

green_app() {
  launch_app "${GREEN_TRACK_NM}" "${GREEN_TRACK_CLR}" "${GREEN_P}" "${BKOUTFILE}" "${GREEN_DCE_IP}" "${PIDDIR}/greentrack.pid" "${SLOW_RATE}" "${ICMP_RATE}" "${MANUAL_MODE}"

#--#  launch_app "${GREEN_TRACK_NM}" "${GREEN_TRACK_CLR}" "${GREEN_P}" "${BKOUTFILE}" "${GREEN_DCE_IP}" "${PIDDIR}/greentrack.pid" "2400" "${MANUAL_MODE}"
}

blue_app() {
  launch_app "${BLUE_TRACK_NM}" "${BLUE_TRACK_CLR}" "${BLUE_P}" "${BKOUTFILE}" "${BLUE_DCE_IP}" "${PIDDIR}/bluetrack.pid" "${SLOW_RATE}" "${ICMP_RATE}" "${MANUAL_MODE}"
}

yellow_app() {
 launch_app "${YELLOW_TRACK_NM}" "${YELLOW_TRACK_CLR}" "${YELLOW_P}" "${BKOUTFILE}" "${YELLOW_DCE_IP}" "${PIDDIR}/yellowtrack.pid" "${SLOW_RATE}" "${ICMP_RATE}" "${MANUAL_MODE}"
}

spare_app() {
  launch_app "${SPARE_TRACK_NM}" "${SPARE_TRACK_CLR}" "${SPARE_P}" "${BKOUTFILE}" "${SPARE_DCE_IP}" "${PIDDIR}/sparetrack.pid" "${SLOW_RATE}" "${ICMP_RATE}" "${MANUAL_MODE}"
}

usage ()
{
  echo $"Usage: $0 [-m] {all|red|blue|green|yellow|spare}" 1>&2
  RETVAL=2
}

#****************************************
# Actually launch the applications
#****************************************
while getopts "m" arg; do
  case "${arg}" in
      m)
        MANUAL_MODE="yes"
        ;;
      *)
        usage
	exit ${RETVAL}
        ;;
  esac
done
shift $((OPTIND-1))

start_track=$1

if [ -z "${start_track}" ] ; then
  usage
  exit ${RETVAL}
fi

case ${start_track} in
    all)
       red_app
       blue_app
       green_app
       yellow_app
       ;;
    testapp)
       test_app
       ;;
    red)
       red_app
       ;;
    blue)
       blue_app
       ;;
    green)
       green_app
       ;;
    yellow)
       yellow_app
       ;;
    spare)
       spare_app
       ;;
    *) usage
       exit ${RETVAL}
       ;;
esac

exit ${RETVAL}
