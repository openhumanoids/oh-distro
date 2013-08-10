export DRC_BASE="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

DRC_SOFTWARE=$DRC_BASE/software
DRC_PATH=$DRC_SOFTWARE # for compatibility
DRC_CONFIG=$DRC_BASE/software/config


setup_drcsim()
{
  source /usr/local/share/drcsim/setup.sh # this will also source setup.sh for ROS and Gazebo
  export LD_LIBRARY_PATH=/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib:$LD_LIBRARY_PATH
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DRC_BASE/ros_workspace:/usr/share/osrf-common-1.0/ros
  export GAZEBO_PLUGIN_PATH=$DRC_SOFTWARE/build/lib:$GAZEBO_PLUGIN_PATH
  export GAZEBO_PLUGIN_PATH=$DRC_BASE/ros_workspace/mit_drcsim_plugins/lib:$GAZEBO_PLUGIN_PATH
  export GAZEBO_MODEL_PATH=$DRC_SOFTWARE/models/mit_gazebo_models:$GAZEBO_MODEL_PATH
  export GAZEBO_MODEL_PATH=$DRC_SOFTWARE/models/mit_gazebo_objects:$GAZEBO_MODEL_PATH
  export VRC_CHEATS_ENABLED=1
}


setup_drc()
{
  export PATH=$PATH:$DRC_SOFTWARE/build/bin
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$DRC_SOFTWARE/build/lib:$DRC_SOFTWARE/build/lib64:$LD_LIBRARY_PATH
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$DRC_SOFTWARE/build/share/java/lcmtypes_drc_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$DRC_SOFTWARE/build/share/java/drake.jar
  export PKG_CONFIG_PATH=$DRC_SOFTWARE/build/lib/pkgconfig:$DRC_SOFTWARE/build/lib64/pkgconfig:$PKG_CONFIG_PATH
  export LCM_URL_DRC_ROBOT="udpm://239.255.76.67:7667?ttl=0"
  export LCM_URL_DRC_BASE="udpm://239.255.76.68:7668?ttl=0"

  # gurobi
  export GUROBI_HOME=$DRC_SOFTWARE/externals/gurobi/gurobi550/linux64
  export PATH=$PATH:$GUROBI_HOME/bin
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib
  export GRB_LICENSE_FILE=$HOME/gurobi.lic

  # python path
  export PYTHONPATH=$PYTHONPATH:$DRC_SOFTWARE/build/lib/python2.7/site-packages:$DRC_SOFTWARE/build/lib/python2.7/dist-packages
}


setup_drcsim
setup_drc


# aliases
alias cddrc='cd $DRC_SOFTWARE'
alias rundrc='bot-procman-sheriff -l $DRC_CONFIG/drc_robot.pmd'
