#!/bin/bash
# Launch roscore
roscore & # If core is already running, this will fail silently
pidcore=$!
sleep 1
# Launch SCS.
$DRC_BASE/software/ipab/config/runscs.bash &
# Get the bash pid.
pidsh=$!
while true; do
	# If SCS crashes and bash script stops, return FAILURE
	if ! kill -0 $pidsh > /dev/null 2>&1; then
		echo "FAILURE - SCS crashed or didn't start."
	        kill $pidcore
		exit 1
	fi
	if [ "$(rostopic info /ihmc_ros/valkyrie/api_command | grep Subscribers)" == "Subscribers: " ] ; then
		# SCS is running now, send commands.
		echo "Shutting down SCS"
                rostopic pub /ihmc_ros/valkyrie/api_command std_msgs/String stop -1 > /dev/null 2>&1
		break
	else
		echo "Waiting for SCS to start ..."
	fi
	sleep 1
done
# Get the java process pid (this is required for shutting down SCS when it becomes unresponsive).
pid="$(ps --ppid $pidsh | grep java | awk '{print $1;}')"
# If SCS closes within 5s, succeeed, fail otherwise.
sleep 5
if ! kill -0 $pid > /dev/null 2>&1; then
	echo "SUCCESS"
        kill $pidcore
	exit 0
else
	kill $pid
	wait $pidsh
        kill $pidcore
	echo "FAILURE - SCS did not shut down! Killing process now."
	exit 1
fi

