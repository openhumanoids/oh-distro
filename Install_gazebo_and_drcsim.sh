#!/bin/bash
GAZEBO_REV=8795
GAZEBO_VERSION=1.8
SIM_REV=2840
MODELS_REV=310

install_sandia_from_src(){
  echo "Installing sandia-hand from src ===================="
  CURRENT_DIR=$PWD
  cd $DRC_BASE/ros_workspace/sandia-hand/  
  ./install.sh
  cd $CURRENT_DIR
  echo " 'find_package(sandia-hand)' should now work in cmake "
  echo "Finished Installing sandia-hand ===================="
}

cd ~/
mkdir ~/gazebo_versions
mkdir ~/gazebo_versions/gazebo_$GAZEBO_REV

echo "Run this script to install the 3 Gazebo Repositories"
echo "You should run it three times in order: gazebo, drcsim, models"
read -p "gazebo, drcsim, models? (g/d/m) " RESP
if [ "$RESP" = "g" ]; then

  make clean -C ~/drc/software/externals/bullet
  echo "CHECKING OUT GAZEBO ====================================="
  hg clone https://bitbucket.org/osrf/gazebo ~/gazebo_versions/gazebo_$GAZEBO_REV/gazebo 
  cd ~/gazebo_versions/gazebo_$GAZEBO_REV/gazebo 
  echo "Applying Specific Revision of Gazebo ===================="
  sleep 2
  hg update -r$GAZEBO_REV
  pwd
  mkdir build
  cd build
  cmake -DPKG_CONFIG_PATH=/opt/ros/fuerte/lib/pkgconfig/:/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib/pkgconfig/ .. 
  echo "Config done on Gazebo, now to build ====================="
  sleep 2
  make -j6
  sudo make install 
  pwd
  cd ~/
  make -C ~/drc/software/externals/bullet
  echo "Finished Installing Gazebo==================="

elif [ "$RESP" = "d" ]; then

  install_sandia_from_src
  source /opt/ros/fuerte/setup.bash
  source /usr/local/share/gazebo-$GAZEBO_VERSION/setup.sh 
  echo "CHECKING OUT DRCSIM ======================="
  sleep 2
  hg clone https://bitbucket.org/osrf/drcsim ~/gazebo_versions/gazebo_$GAZEBO_REV/drcsim 
  cd ~/gazebo_versions/gazebo_$GAZEBO_REV/drcsim 
  echo "Applying Specific Revision of DRCSIM =================="
  sleep 2
  hg update -r$SIM_REV
  mkdir build
  cd build
  cmake -Dsandia-hand_DIR=$DRC_BASE/ros_workspace/sandia-hand/build ..
  echo "cmake done on DRCSIM, now to build ====================="
  sleep 2
  make -j6
  sudo make install 
  cd ../..
  echo "Finished Installing DRCSIM==================="
  
else
  echo "GETTING A NEW VERSION OF ~/gazebo/.models ====================================="
  rm ~/.gazebo/models -Rf
  hg clone https://bitbucket.org/osrf/gazebo_models ~/.gazebo/models 
  cd ~/.gazebo/models
  hg pull
  hg update -r$MODELS_REV
  echo "Finished Installing ~/gazebo/.models ==================="

fi



