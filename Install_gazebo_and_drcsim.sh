#!/bin/bash
GAZEBO_REV=8795
GAZEBO_VERSION=1.8
SIM_REV=2840
MODELS_REV=310

make_parallel=-j6

# get the path to DRC svn repo (where this script is located)
cd `dirname $0`
drc_dir=`pwd`
work_dir=$drc_dir/software/build/drcsim-and-gazebo-builds


clean_build_dir()
{
  rm -rf $work_dir
  mkdir -p $work_dir
}

build_sandia_hand()
{
  echo "Installing sandia-hand from src ===================="

  source /opt/ros/fuerte/setup.bash
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/source/drc-trunk/ros_workspace:/usr/share/osrf-common-1.0/ros
  rosmake sandia_hand_driver --pre-clean

  cd $drc_dir/ros_workspace/sandia-hand
  ./install.sh

  echo "Finished Installing sandia-hand ===================="
}


build_gazebo()
{
  echo "CHECKING OUT GAZEBO ====================================="
  hg clone https://bitbucket.org/osrf/gazebo $work_dir/gazebo

  echo "Applying Specific Revision of Gazebo ===================="
  cd $work_dir/gazebo
  hg update -r$GAZEBO_REV

  echo "Configure and build Gazebo ====================="
  cd ..
  rm -rf gazebo-build
  mkdir gazebo-build
  cd gazebo-build
  cmake -DPKG_CONFIG_PATH=/opt/ros/fuerte/lib/pkgconfig/:/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib/pkgconfig/ ../gazebo
  make $make_parallel
  sudo make install

  echo "Performing a clean build of bullet ====================="
  cd $drc_dir/software/externals/bullet
  make clean
  make $make_parallel

  echo "Finished Installing Gazebo==================="
}


build_drcsim()
{
  source /opt/ros/fuerte/setup.bash
  source /usr/local/share/gazebo-$GAZEBO_VERSION/setup.sh

  echo "CHECKING OUT DRCSIM ======================="
  hg clone https://bitbucket.org/osrf/drcsim $work_dir/drcsim


  echo "Applying Specific Revision of DRCSIM =================="
  cd $work_dir/drcsim
  hg update -r$SIM_REV

  echo "Configure and build DRCSIM ====================="
  cd ..
  rm -rf drcsim-build
  mkdir drcsim-build
  cd drcsim-build
  cmake  -Dsandia-hand_DIR=$drc_dir/ros_workspace/sandia-hand/build ../drcsim
  make $make_parallel
  sudo make install

  echo "Finished Installing DRCSIM==================="
}


download_models()
{
  echo "GETTING A NEW VERSION OF ~/gazebo/.models ====================================="
  rm ~/.gazebo/models -Rf
  hg clone https://bitbucket.org/osrf/gazebo_models ~/.gazebo/models
  cd ~/.gazebo/models
  hg pull
  hg update -r$MODELS_REV
  echo "Finished Installing ~/gazebo/.models ==================="
}

clean_build_dir
build_sandia_hand
build_gazebo
build_drcsim
download_models


