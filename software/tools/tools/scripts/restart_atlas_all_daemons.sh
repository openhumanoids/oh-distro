#!/bin/bash

# field scripts
ssh field -t 'bash -ic "sudo \${DRC_BASE}/software/tools/tools/scripts/atlas_sync_time.sh; \${DRC_BASE}/software/tools/tools/scripts/restart_atlas_deputies.sh; \${DRC_BASE}/software/tools/tools/scripts/restart_atlas_field_networking.sh"'

# network shaper
killall -qw drc-network-shaper
screen -D -m -S shaper-link2 drc-network-shaper -r base -c drc_robot.cfg -i link2 &
screen -D -m -S shaper-link3 drc-network-shaper -r base -c drc_robot.cfg  -i link3 -l -p /home/drc/logs/raw/shaper &

until pids=$(pidof drc-network-shaper); do sleep 0.1; done
echo started network shaper on `hostname`
