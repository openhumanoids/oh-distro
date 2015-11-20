#!/bin/bash

# lcm tunnel
${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_tunnel.sh server
ssh atlas0 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_tunnel.sh"'

# lcm bridges
ssh atlas0 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_bridge.sh"'
ssh atlas1 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_bridge.sh"'
ssh atlas2 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_bridge.sh"'

# network shaper
killall -qw drc-network-shaper
screen -D -m -S shaper-link3 drc-network-shaper -r robot -c drc_robot.cfg  -i link3 -l -p /home/drc/logs/raw/shaper &
screen -D -m -S shaper-link2 drc-network-shaper -r robot -c drc_robot.cfg  -i link2 &

until pids=$(pidof drc-network-shaper); do sleep 0.1; done
echo started network shaper on `hostname`
