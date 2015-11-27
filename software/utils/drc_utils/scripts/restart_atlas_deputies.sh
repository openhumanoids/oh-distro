#!/bin/bash

ssh atlas0 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_deputy.sh"'
ssh atlas1 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_deputy.sh"'
ssh atlas2 -t 'bash -ic "\${DRC_BASE}/software/utils/drc_utils/scripts/atlas_restart_deputy.sh"'
echo deputies complete

# time sync (atlas0 syncs to server, atlas1+2 sync to atlas0)
ssh atlas0 -t 'bash -ic "sudo \${DRC_BASE}/software/utils/drc_utils/scripts/atlas_sync_time.sh"'
sleep 5
ssh atlas1 -t 'bash -ic "sudo \${DRC_BASE}/software/utils/drc_utils/scripts/atlas_sync_time.sh"'
ssh atlas2 -t 'bash -ic "sudo \${DRC_BASE}/software/utils/drc_utils/scripts/atlas_sync_time.sh"'
echo time sync complete
