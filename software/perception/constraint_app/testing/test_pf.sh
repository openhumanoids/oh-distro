#!/bin/bash

bot-param-server $DRC_BASE/software/config/drc_robot_02.cfg &
affordance_server  -r robot & 
drc-viewer &

sleep 1

ca_test &
ID_CA_TEST=$!

matlab -nosplash -r "cd matlab; track_pf();"
RETCODE_MATLAB=$?

wait $ID_CA_TEST
RETCODE_CA_TEST=$?

killall bot-param-server affordance_server drc-viewer

echo Return code from matlab was $RETCODE_MATLAB
echo Return code from ca_test was $RETCODE_CA_TEST

if [ $RETCODE_MATLAB != 0 ]
then exit $RETCODE_MATLAB
fi

if [ $RETCODE_CA_TEST != 0 ]
then exit $RETCODE_CA_TEST
fi