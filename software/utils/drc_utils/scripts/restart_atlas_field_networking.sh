#!/bin/bash

# lcm tunnel
${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_tunnel.sh server
ssh atlas0 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_tunnel.sh"'

# lcm bridges
ssh atlas0 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_bridge.sh"'
ssh atlas1 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_bridge.sh"'
ssh atlas2 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_bridge.sh"'
