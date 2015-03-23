#!/bin/bash

# field scripts
ssh field -t 'bash -ic "\${DRC_BASE}/software/tools/tools/scripts/restart_atlas_deputies.sh; \${DRC_BASE}/software/tools/tools/scripts/restart_atlas_field_networking.sh"'

# network shaper
killall -q drc-network-shaper
screen -D -m -S shaper-link2 drc-network-shaper -r base -c drc_robot.cfg -i link2 &
screen -D -m -S shaper-link3 drc-network-shaper -r base -c drc_robot.cfg  -i link3 &
