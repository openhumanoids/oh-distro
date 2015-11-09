#!/bin/bash
# Installs dependencies

sudo add-apt-repository -y ppa:goby-dev/ppa
sudo add-apt-repository -y ppa:tes/drc

sudo apt-get update
sudo apt-get upgrade -y

sudo apt-get install -y git gitk git-gui terminator build-essential cmake debhelper freeglut3-dev gtk-doc-tools libboost-filesystem-dev libboost-iostreams-dev libboost-program-options-dev libboost-random-dev libboost-regex-dev libboost-signals-dev libboost-system-dev libboost-thread-dev libcurl4-openssl-dev libfreeimage-dev libgoby2-dev libgoby2 libdccl3-dev libdccl3 libglew-dev libgtkmm-2.4-dev libltdl-dev libgsl0-dev libportmidi-dev libprotobuf-dev libprotoc-dev libqt4-dev libqwt-dev libtar-dev libtbb-dev libtinyxml-dev libxml2-dev ncurses-dev pkg-config protobuf-compiler python-matplotlib libvtk5.8 libvtk5-dev libvtk5-qt4-dev libqhull-dev python-pygame doxygen mercurial libglib2.0-dev openjdk-6-jdk python-dev gfortran f2c libf2c2-dev spacenavd libspnav-dev python-numpy python-scipy python-yaml python-vtk python-pip libgmp3-dev libblas-dev liblapack-dev libv4l-dev subversion libxmu-dev libusb-1.0-0-dev python-pymodbus graphviz curl libwww-perl libterm-readkey-perl libx264-dev libopenni-dev swig

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

sudo apt-get install -y ros-indigo-desktop-full
sudo apt-get install -y ros-indigo-moveit-full

cd ~
wget https://github.com/lcm-proj/lcm/releases/download/v1.1.0/lcm-1.1.0.zip
unzip lcm-1.1.0.zip
cd lcm-1.1.0
./configure
make
sudo make install
