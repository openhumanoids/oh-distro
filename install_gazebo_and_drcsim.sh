#!/bin/bash
GAZEBO_TAG=gazebo_2.2
SDFORMAT_TAG=sdf_1.4
DRCSIM_TAG=drcsim_3.1.1
MODELS_REV=467

make_parallel=-j6

work_dir=$DRC_BASE/software/build/drcsim-and-gazebo-builds

clean_build_dir()
{
  rm -rf $work_dir
  mkdir -p $work_dir
}

build_sdformat()
{
  echo "CHECKING OUT sdformat ====================================="
  hg clone https://bitbucket.org/osrf/sdformat $work_dir/sdformat

  echo "Applying Specific Revision of sdformat ===================="
  cd $work_dir/sdformat
  hg up $SDFORMAT_TAG

  echo "Configure and build sdformat ====================="
  rm -rf build
  mkdir build
  cd build
  cmake ../
  make $make_parallel
  sudo make install

  echo "Finished Installing sdformat ==================="
}

build_gazebo()
{
  echo "CHECKING OUT GAZEBO ====================================="
  hg clone https://bitbucket.org/osrf/gazebo $work_dir/gazebo

  # echo "Removing bullet ====================="
  # cd $drc_dir/software/externals/bullet
  # make clean

  echo "Applying Specific Revision of Gazebo ===================="
  cd $work_dir/gazebo
  hg up $GAZEBO_TAG

  echo "Configure and build Gazebo ====================="
  rm -rf build
  mkdir build
  cd build
  cmake ../ -DCMAKE_INSTALL_PREFIX=/usr/local
  make $make_parallel
  sudo make install

  # echo "Performing a clean build of bullet ====================="
  # cd $drc_dir/software/externals/bullet
  # make $make_parallel

  echo "Finished Installing Gazebo==================="
}


build_sandia_hand()
{
  echo "Installing sandia-hand from src ===================="

  source /opt/ros/groovy/setup.sh
#  source /usr/local/share/gazebo/setup.sh
 
  echo "CHECKING OUT sandia-hand ======================="
	hg clone https://bitbucket.org/osrf/sandia-hand $work_dir/sandia-hand

  echo "Configure and build sandia-hand ====================="
  cd $work_dir/sandia-hand
  rm -rf build
  mkdir build
  cd build
  cmake ../ -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
  make $make_parallel
  sudo make install

  echo "Finished Installing sandia-hand ===================="
}


build_osrf_common()
{
  source /opt/ros/groovy/setup.sh
#  source /usr/local/share/gazebo/setup.sh
 
  echo "CHECKING OUT OSRF-COMMON ======================="
  hg clone https://bitbucket.org/osrf/osrf-common $work_dir/osrf-common

  echo "Configure and build OSRF-COMMON ====================="
  cd $work_dir/osrf-common
  rm -rf build
  mkdir build
  cd build
  cmake ../ -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
  make $make_parallel
  sudo make install

  echo "Finished Installing OSRF-COMMON ==================="
}


build_gazebo_ros_pkgs()
{
  source /opt/ros/groovy/setup.sh
 
  echo "CHECKING OUT GAZEBO-ROS-PKGS ======================="
  git clone https://github.com/ros-simulation/gazebo_ros_pkgs $work_dir/gazebo_ros_pkgs

  echo "Configure and build GAZEBO-ROS-PKGS ====================="
  cd $work_dir/gazebo_ros_pkgs
	# build gazebo_msgs
	cd gazebo_msgs
  rm -rf build
	mkdir build
	cd build
	cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
	make $make_parallel
	sudo make install

	# build gazebo_plugins
	cd ../../gazebo_plugins
  rm -rf build
	mkdir build
	cd build
	cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
	make $make_parallel
	sudo make install

	# gazebo_ros
	cd ../../gazebo_ros
  rm -rf build
	mkdir build
	cd build
	cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO
	make $make_parallel
	sudo make install

  echo "Finished Installing GAZEBO-ROS-PKGS ==================="
}


build_drcsim()
{
  source /opt/ros/groovy/setup.sh
#  source /usr/local/share/gazebo/setup.sh
 
  echo "CHECKING OUT DRCSIM ======================="
  hg clone https://bitbucket.org/osrf/drcsim $work_dir/drcsim

  echo "Applying Specific Revision of DRCSIM =================="
  cd $work_dir/drcsim
  hg up $DRCSIM_TAG

  echo "Configure and build DRCSIM ====================="
  rm -rf build
  mkdir build
  cd build
  cmake ../ -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCMAKE_INSTALL_PREFIX=/usr/local
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
build_sdformat
build_gazebo
build_osrf_common
build_sandia_hand
build_gazebo_ros_pkgs
build_drcsim
download_models



