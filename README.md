MIT DRC
=======


Background
==========

[MIT DRC youtube video](https://www.youtube.com/watch?v=FiH4pId-zFM)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=FiH4pId-zFM
" target="_blank"><img src="http://img.youtube.com/vi/FiH4pId-zFM/0.jpg" 
alt="MIT DRC youtube video" width="480" height="360" border="10" /></a>


Software Build Instructions
===========================

This README describes how to download and build the MIT DRC source code and
how to satisfy 3rd party dependencies.


System Requirements
===================

These instructions are written for Ubuntu 12.04 64-bit.


Download Instructions
=====================

The DRC source code is stored in a Git repository.  To download the source code
you may need to first install Git on your system:

```
sudo apt-get install git gitk git-gui
```

Next, you may need permission to access the git repository on Github.  To do so,
create a github account

Download the repository with the ```git clone``` command:

```
git clone git@github.com:mitdrc/drc.git
```

Initialize the Drake submodule:

```
cd drc
git submodule update --init
```

Add the *stage* remote.  The *stage* remote is the location where shared
branches are stored among developers.


```
cd drc
git remote add stage git@github.com:drcbot/drc.git
git fetch stage
```


Dependencies
============

There are required 3rd party dependencies that can be satisfied by installing
packages on Ubuntu.  Install with the following commands:


```
sudo add-apt-repository ppa:goby-dev/ppa ppa:tes/drc
sudo apt-get update

sudo apt-get install build-essential cmake debhelper freeglut3-dev gtk-doc-tools libboost-filesystem-dev libboost-iostreams-dev libboost-program-options-dev libboost-random-dev libboost-regex-dev libboost-signals-dev libboost-system-dev libboost-thread-dev libcurl4-openssl-dev libfreeimage-dev libgoby2-dev libglew-dev libgtkmm-2.4-dev libltdl-dev libgsl0-dev libprotobuf-dev libprotoc-dev libqt4-dev libqwt-dev libtar-dev libtbb-dev libtinyxml-dev libxml2-dev ncurses-dev openni-dev pkg-config protobuf-compiler python-matplotlib libvtk5.8 libvtk5-dev libqhull-dev python-pygame doxygen mercurial libglib2.0-dev openjdk-6-jdk python-dev gfortran f2c libf2c2-dev spacenavd libspnav-dev
```


LCM Dependency
==============

LCM is a required dependency.  Install LCM v0.9 from source with the following
commands:

```
wget https://lcm.googlecode.com/files/lcm-0.9.1.tar.gz
tar -xzf lcm-0.9.1.tar.gz
cd lcm-0.9.1
./configure
make
sudo make install
```


Install Matlab
==============

Install Matlab r2012b 64bit from TIG:

https://tig.csail.mit.edu/software-distribution/Linux/MathWorks/R2012b/R2012b-linux64.zip

Follow the instructions on the DRC wiki for Matlab install with activation code:

https://groups.csail.mit.edu/rvsn/wiki/index.php?title=Installing_MIT_DRC#Install_MATLAB_and_Drake


GUROBI License
==============

Follow the install instructions on the wiki.  The wiki page includes instructions
for setting gurobi related environment variables in ~/.bashrc.  You should skip
these steps.  Instead, follow the steps in this README under Environment Setup.


https://groups.csail.mit.edu/rvsn/wiki/index.php?title=Installing_GUROBI


ROS Dependencies
================

The ROS dependencies are optional.  Install with the following commands:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu precise main" > /etc/apt/sources.list.d/drc-latest.list'

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-fuerte-common-msgs ros-fuerte-geometry ros-fuerte-image-common ros-fuerte-pr2-mechanism ros-fuerte-std-msgs ros-fuerte-urdfdom ros-fuerte-visualization-common ros-fuerte-pr2-controllers ros-fuerte-geometry-experimental ros-fuerte-robot-model-visualization ros-fuerte-image-pipeline ros-fuerte-vision-opencv ros-fuerte-image-transport-plugins ros-fuerte-qt-ros python-rosdep python-rosinstal
```


Environment Setup
=================

The behavior of certain build steps can be affected by environment variables,
so you should setup your environment before starting the build.  The DRC
environment is setup by sourcing the file *drc/software/config/drc_environment.sh*.
Typically, users will source this file automatically in their ~/.bashrc file
by adding this line to ~/.bashrc:

```
source /path-to/drc/software/config/drc_environment.sh
```

If you have already done this, make sure your ~/.bashrc contains the correct path
to the drc_environment.sh file in the drc source code directory that you just
cloned with git.


Matlab Environment Setup
========================

Create a file ~/Documents/MATLAB/startup.m that contains the line:

```
run([getenv('DRC_BASE'), '/software/build/config/drc_control_setup.m'])
```


Build instructions
==================

Make sure you have sourced the drc_environment.sh file to setup the DRC environment
prior to building.  If you did not source the file automatically in ~/.bashrc, then
do so now with the following command:

```
cd drc
source software/config/drc_environment.sh
```

Run make to build externals and then the main codebase:

```
cd software/externals
make
cd ..
make
```
