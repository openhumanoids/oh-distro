#!/bin/bash
for i in `seq 1 10`;
do
  echo "starting run $i of 10"
	bot-procman-sheriff -l --on-script-complete exit ~/drc/software/config/drc_robot.pmd systemtest 
  sleep 5  
done
