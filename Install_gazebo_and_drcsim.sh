#!/bin/bash
GAZEBO_REV=7503
GAZEBO_VERSION=1.7
SIM_REV=2006
MODELS_REV=212

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
  make -j8
  sudo make install 
  pwd
  cd ~/
  make -C ~/drc/software/externals/bullet
  echo "Finished Installing Gazebo==================="

elif [ "$RESP" = "d" ]; then

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
  cmake ..
  echo "cmake done on DRCSIM, now to build ====================="
  sleep 2
  make
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



