#!/bin/bash
# drc-lcm-logger.sh
# Author: Maurice Fallon
# Writes LCM logs to a specific directory, but adds time as extension

date_str=$(date +"lcmlog-%Y-%m-%d.%H:%M")
date_str="$DRC_BASE/logs/$date_str"
lcm-logger $date_str
