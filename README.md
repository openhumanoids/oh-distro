# ipab-distro
This repository brings together modules for motion planning, control, interfaces with Valkyrie, Atlas, KUKA LWR and other robots, state estimation and other sensing modules.

## Repository structure

* [ipab-slmc/ipab-distro](https://github.com/ipab-slmc/ipab-distro)
  * [mitdrc/drc](https://github.com/mitdrc/drc)
    * [RobotLocomotion/director](https://github.com/RobotLocomotion/director)
    * [mitdrc/drake](https://github.com/mitdrc/drake)
      * [RobotLocomotion/cmake ](https://github.com/RobotLocomotion/cmake)
    * [mitdrc/pronto](https://github.com/mitdrc/pronto)
  * [ipab-slmc/ipab-ros-workspace](https://github.com/ipab-slmc/ipab-ros-workspace)
    * [ipab-slmc/ipab_lwr_msgs](https://github.com/ipab-slmc/ipab_lwr_msgs)
    * [IPAB-SLMC/exotica](https://bitbucket.org/IPAB-SLMC/exotica) (Bitbucket)
    * [IPAB-SLMC/exotica_json](https://bitbucket.org/IPAB-SLMC/exotica_json) (Bitbucket)

## To get started

Check out the required code:

```
cd ~
git clone git@github.com:ipab-slmc/ipab-distro.git
cd ipab-distro
git submodule update --init --recursive
cd drc
```

The next step is to build the DRC repository. Follow the instructions starting at 'Add the sandbox remote' here:

* https://github.com/mitdrc/drc



### ROS workspace

The final step is build the IPAB ROS workspace. The following should be added to your bashrc:

    source ~/ipab-distro/drc/software/config/drc_environment.sh
    source /opt/ros/indigo/setup.bash
    source $DRC_BASE/catkin_ws/drc_translators/devel/setup.bash

The devel/setup.bash file will be created after you compile the catkin workspace. You may want to source this file after the first compilation.

Install the required ROS packages:

    ros-indigo-tf ros-indigo-image-transport ros-indigo-tf-conversions ros-indigo-image-geometry ros-indigo-cv-bridge ros-indigo-std-srvs

The catkin workspace is located in source catkin_ws/drc_translators. The workspace is compiled separately by running

    catkin_make -DCATKIN_BUILD_MODE=RelWithDebInfo
 
Run this command in the workspace folder. The release mode will greatly improve performance, especially when using EXOTica.

## To Contribute

 1. Create a new branch with your feature in the corresponding submodule
 1. Make a pull request against the corresponding submodule
 1. Once the PR of the corresponding submodule is accepted, create a branch of ipab-distro and forward the submodule tip, and make a pull request against ipab-distro/master with a working combination of submodules


## Other hints

### Optional modules
This distro does not include the Kuka LWR and Schunk SDH repositories that are only needed on the respective computers running these components. These are at the time of writing ``wendy`` for the Kuka LWR driver and ``mindy`` for the Schunk SDH driver. The source code for the packages can be found here: [Kuka LWR](https://bitbucket.org/IPAB-SLMC/kuka-lwr/), [Schunk SDH](https://bitbucket.org/IPAB-SLMC/schunk-sdh).

### Easy manual sourcing

Alternatively, if you wish to source the environemt in each terminal window manually, add the following line to your ~/.bash_aliases

    alias init_drc="source ~/dev/drc/software/config/drc_environment.sh;source /opt/ros/indigo/setup.bash;source $DRC_BASE/catkin_ws/drc_translators/devel/setup.bash;"

