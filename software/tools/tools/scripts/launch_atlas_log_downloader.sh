#!/bin/bash

source $HOME/drc/software/config/drc_environment.sh

mkdir -p $HOME/logs/atlas
cd $HOME/logs/atlas

python $ATLAS_ROBOT_INTERFACE/tools/atlas_log_downloader.py

