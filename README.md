# ipab-distro
This repository checks out working tandems of the ``drc`` and ``ipab-ros-workspace`` codebases. 

To get started:

```
cd ~
git clone git@github.com:ipab-slmc/ipab-distro.git
cd ipab-distro
git submodule update --init --recursive
```

To contribute:

 1. Create a new branch with your feature in the corresponding submodule
 1. Make a pull request against the corresponding submodule
 1. Once the PR of the corresponding submodule is accepted, create a branch of ipab-distro and forward the submodule tip, and make a pull request against ipab-distro/master with a working combination of submodules


## Optional modules
This distro does not include the Kuka LWR and Schunk SDH repositories that are only needed on the respective computers running these components. These are at the time of writing ``wendy`` for the Kuka LWR driver and ``mindy`` for the Schunk SDH driver. The source code for the packages can be found here: [Kuka LWR](https://bitbucket.org/IPAB-SLMC/kuka-lwr/), [Schunk SDH](https://bitbucket.org/IPAB-SLMC/schunk-sdh).



##ROS Catkin workspace

Missing repositories (June 2015):

[submodule "catkin_ws/drc_translators/src/exotica_json"]
        path = catkin_ws/drc_translators/src/exotica_json
        url = git@bitbucket.org:IPAB-SLMC/exotica_json.git
[submodule "catkin_ws/drc_translators/src/exotica"]
        path = catkin_ws/drc_translators/src/exotica
        url = git@bitbucket.org:IPAB-SLMC/exotica.git

Required ROS packages:

::

    ros-indigo-tf ros-indigo-image-transport ros-indigo-tf-conversions ros-indigo-image-geometry ros-indigo-cv-bridge ros-indigo-std-srvs

Add the following to bashrc for IPAB's ROS Catkin workspace

::

    source /opt/ros/indigo/setup.bash
    source $DRC_BASE/catkin_ws/drc_translators/devel/setup.bash

Alternatively, if you wish to source the environemt in each terminal window manually, add the following line to your ~/.bash_aliases

::

    alias init_drc="source ~/dev/drc/software/config/drc_environment.sh;source /opt/ros/indigo/setup.bash;source $DRC_BASE/catkin_ws/drc_translators/devel/setup.bash;"

The devel/setup.bash file will be created after you compile the catkin workspace. You may want to source this file after the first compilation.

The catkin workspace is located in source catkin_ws/drc_translators. The workspace is compiled separately by running

::

    catkin_make -DCATKIN_BUILD_MODE=RelWithDebInfo
 
Run this command in the worspace folder. The release mode will greatly improve performance, especially when using EXOTica. Any additional ROS packages can be either put in this folder or symlinks can be created.
