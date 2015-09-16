#!/bin/bash
# Checks out the Edinburgh distribution and sets everything up

# Check out all code, add sandboxes, fetch all branches
cd ~
git clone git@github.com:ipab-slmc/ipab-distro.git
cd ipab-distro
git submodule update --init --recursive
cd drc

git remote add sandbox git@github.com:drcbot/drc.git
git fetch sandbox

cd software/ddapp
git remote add sandbox git@github.com:mitdrc/director.git
git fetch sandbox

# Sets up shortcuts
echo "alias init_drc='source ~/ipab-distro/drc/software/config/drc_environment.sh'" >> ~/.bashrc
# source ~/ipab-distro/ipab-ros-workspace/devel/setup.bash

echo "run([getenv('DRC_BASE'), '/software/build/config/drc_control_setup.m'])" >> ~/Documents/MATLAB/startup.m # TODO check whether already there