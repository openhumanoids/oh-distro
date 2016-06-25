#!/bin/bash

# lcm tunnel
ssh link02 -t 'bash -ic "~/openhumanoids/oh-distro/software/utils/drc_utils/scripts/valkyrie_restart_tunnel.sh"'
/home/val/openhumanoids/oh-distro/software/utils/drc_utils/scripts/valkyrie_restart_tunnel.sh server

# do the same for zelda02, currently only using link02
# ssh link02 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/valkyrie_restart_tunnel.sh"'
