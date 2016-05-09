#!/bin/bash

ssh link02 -t 'bash -ic "/home/val/openhumanoids/oh-distro/software/utils/drc_utils/scripts/valkyrie_restart_deputy.sh"'

/home/val/openhumanoids/oh-distro/software/utils/drc_utils/scripts/valkyrie_restart_deputy.sh

# currently only running with a deputy on link02, bring up zelda02 later
# ssh zelda02 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/valkyrie_restart_deputy.sh"'
echo deputies complete

