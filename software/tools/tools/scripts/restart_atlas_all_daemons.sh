#!/bin/bash

# field scripts
ssh field -t 'bash -ic "\${DRC_BASE}/software/tools/tools/scripts/restart_atlas_deputies.sh; \${DRC_BASE}/software/tools/tools/scripts/restart_atlas_field_networking.sh"'

# network shaper
killall -qw drc-network-shaper
screen -D -m -S shaper-link2 drc-network-shaper -r base -c drc_robot.cfg -i link2 &
screen -D -m -S shaper-link3 drc-network-shaper -r base -c drc_robot.cfg  -i link3 &

until pids=$(pidof drc-network-shaper); do sleep 0.1; done
echo started network shaper on `hostname`

# time sync (atlas0 syncs to server, atlas1+2 sync to atlas0)
ssh atlas0 -t 'bash -ic "sudo \${DRC_BASE}/software/tools/tools/scripts/atlas_sync_time.sh"'
ssh atlas1 -t 'bash -ic "sudo \${DRC_BASE}/software/tools/tools/scripts/atlas_sync_time.sh"'
ssh atlas2 -t 'bash -ic "sudo \${DRC_BASE}/software/tools/tools/scripts/atlas_sync_time.sh"'
echo time sync complete
