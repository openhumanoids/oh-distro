#!/bin/bash

ssh atlas0 -t 'bash -ic "\${DRC_BASE}/software/tools/tools/scripts/atlas_restart_deputy.sh"'
ssh atlas1 -t 'bash -ic "\${DRC_BASE}/software/tools/tools/scripts/atlas_restart_deputy.sh"'
ssh atlas2 -t 'bash -ic "\${DRC_BASE}/software/tools/tools/scripts/atlas_restart_deputy.sh"'
