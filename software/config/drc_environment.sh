

# This function sets the DRC_BASE environment variable to be the DRC
# root directory.  The path to the root directory is computed using
# the absolute path of this bash file.  Due to symlinks, this bash
# file can be located in $DRC_BASE/software/config or
# $DRC_BASE/software/build/config.
set_drc_base()
{
  # may be a relative or absolute path
  environmentFile=${BASH_SOURCE[0]}

  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$environmentFile")/.." && pwd)"

  # different cases for software/config or software/build/config
  case "$(basename $configParentDir)" in
    "software") export DRC_BASE=$(dirname $configParentDir);;
    "build") export DRC_BASE=$(dirname $(dirname $configParentDir));;
    *) echo "Warning: DRC environment file is stored in unrecognized location: $environmentFile";;
  esac
}

setup_drcsim()
{
  source /usr/local/share/drcsim/setup.sh # this will also source setup.sh for ROS and Gazebo
  export LD_LIBRARY_PATH=/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib:$LD_LIBRARY_PATH
  export ROS_PACKAGE_PATH=$DRC_BASE/ros_workspace:$ROS_PACKAGE_PATH
  #:/usr/share/osrf-common-1.0/ros
  export GAZEBO_PLUGIN_PATH=$DRC_BASE/software/build/lib:$GAZEBO_PLUGIN_PATH
  export GAZEBO_PLUGIN_PATH=$DRC_BASE/ros_workspace/mit_drcsim_plugins/lib:$GAZEBO_PLUGIN_PATH
  export GAZEBO_MODEL_PATH=$DRC_BASE/software/models/mit_gazebo_models:$GAZEBO_MODEL_PATH
  export GAZEBO_MODEL_PATH=$DRC_BASE/software/models/mit_gazebo_objects:$GAZEBO_MODEL_PATH
  export VRC_CHEATS_ENABLED=1
}


setup_drc()
{
  export PATH=$PATH:$DRC_BASE/software/build/bin
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$DRC_BASE/software/build/lib:$DRC_BASE/software/build/lib64:$LD_LIBRARY_PATH
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$DRC_BASE/software/build/share/java/lcmtypes_drc_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$DRC_BASE/software/build/share/java/drake.jar
  export PKG_CONFIG_PATH=$DRC_BASE/software/build/lib/pkgconfig:$DRC_BASE/software/build/lib64/pkgconfig:$PKG_CONFIG_PATH
  export LCM_URL_DRC_ROBOT="udpm://239.255.76.67:7667?ttl=0"
  export LCM_URL_DRC_BASE="udpm://239.255.76.68:7668?ttl=0"

  # gurobi
  export GUROBI_HOME=$DRC_BASE/software/externals/gurobi/gurobi550/linux64
  export PATH=$PATH:$GUROBI_HOME/bin
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib
  export GRB_LICENSE_FILE=$HOME/gurobi.lic

  # python path
  export PYTHONPATH=$PYTHONPATH:$DRC_BASE/software/build/lib/python2.7/site-packages:$DRC_BASE/software/build/lib/python2.7/dist-packages

  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"
}


set_drc_base
setup_drcsim
setup_drc


# aliases
alias cddrc='cd $DRC_BASE/software'
alias rundrc='bot-procman-sheriff -l $DRC_BASE/software/config/drc_robot.pmd'
