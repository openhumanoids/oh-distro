#!/bin/bash

TEST_NAME=$1
PROCMAN_SCRIPT=$2
MAX_WAIT=$3

# Result file
export SYSTEMTEST_RESULT_FILE=/tmp/test_test$TEST_NAME.res
# Launch roscore
roscore & # If core is already running, this will fail silently.
pidcore=$!
sleep 1
# Launch SCS.
$DRC_BASE/software/ipab/config/runscs.bash &
# Get the bash pid.
pidscs=$!
director=0
sleep 1
pid="$(ps --ppid $pidscs | grep java | awk '{print $1;}')"

echo "0" > $SYSTEMTEST_RESULT_FILE
T=$MAX_WAIT

while [ $T -gt 0 ]; do
	if ! kill -0 $pidscs > /dev/null 2>&1; then
		echo "FAILURE - SCS crashed or didn't start."
	        kill $pidcore > /dev/null 2>&1
		exit 1
	fi
        if [[ $director -eq 1 ]]; then
		# When sheriff stops, check the status value
		if ! kill -0 $pidsh > /dev/null 2>&1; then
			if [[ "$(cat $SYSTEMTEST_RESULT_FILE)" -eq 0 ]]; then
				echo "FAILURE - Sheriff crashed."
				kill $pidcore > /dev/null 2>&1
				kill $pid > /dev/null 2>&1
				exit 1
			else
				echo "SUCCESS"
				kill $pid > /dev/null 2>&1
				kill $pidcore > /dev/null 2>&1
				exit 0
			fi
		else
			echo "Waiting for Sheriff to finish ..."
		fi
	else
		if [ "$(rostopic info /ihmc_ros/valkyrie/api_command | grep Subscribers)" == "Subscribers: " ] ; then
			# SCS is running now, send commands.
			echo "SCS is ready, starting Director"
			pid="$(ps --ppid $pidscs | grep java | awk '{print $1;}')"
			bot-procman-sheriff -l -n $PROCMAN_SCRIPT start &
			# Get the sheriff's pid.
			pidsh=$!
			director=1
		else
			echo "Waiting for SCS to start ..."
		fi
	fi
	sleep 1
	T=`expr $T - 1`
done

kill $pidscs > /dev/null 2>&1
kill $pidsh > /dev/null 2>&1
kill $pid > /dev/null 2>&1
kill $pidcore > /dev/null 2>&1
echo "FAILURE - time out"
exit 1
