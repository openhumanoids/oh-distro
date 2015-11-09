#!/bin/bash
# Result file
export SYSTEMTEST_RESULT_FILE=/tmp/test_testexoticaLWR.res
# Launch roscore
roscore & # If core is already running, this will fail silently.
pidcore=$!
sleep 1
# Launch Drake Designer.

echo "0" > $SYSTEMTEST_RESULT_FILE
bot-procman-sheriff -l -n $DRC_BASE/../tests/systemtests/src/exoticaLWR/kuka_lwr_exotica.pmd start &
# Get the sheriff's pid.
pidsh=$!
while true; do
	# When sheriff stops, check the status value
	if ! kill -0 $pidsh > /dev/null 2>&1; then
		if [[ "$(cat $SYSTEMTEST_RESULT_FILE)" -eq 0 ]]; then
			echo "FAILURE - Sheriff crashed."
			kill $pidcore
			exit 1
		else
			echo "SUCCESS"
			kill $pidcore
			exit 0
		fi
	fi
	echo "Waiting for Sheriff to finish ..."
	sleep 1
done

kill $pidcore
