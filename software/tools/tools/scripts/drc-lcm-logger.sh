#!/bin/bash
# drc-lcm-logger.sh
# Author: Maurice Fallon
# Writes LCM logs to a specific directory, but adds time as extension
#echo "|$1|"

date_str=$(date +"lcmlog-%Y-%m-%d-%H-%M")
date_str="/media/logs/raw/$date_str"


if [ -n "$1" ]; then
  #echo "1 is not empty"
  date_str=$date_str"-$1" 
fi
#echo $date_str
lcm-logger $date_str
