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

## Basics (Git)
All our source code is stored in Git repositories. Please ensure that you have git installed on your system:
```
sudo apt-get install git gitk git-gui
```

## Getting Access
You may need permission to access the git repositories hosted on GitHub and Bitbucket. To do so, [create a GitHub account](https://github.com/join) as well as a [Bitbucket account](https://bitbucket.org/account/signup/) if you don't already have one.

Next, add your public SSH key to your GitHub ([on this page](https://github.com/settings/ssh/)) and Bitbucket accounts (Click on your profile picture in the top right corner, then Manage Account, then SSH keys) so that you can easily push and pull over SSH. Read the [generating ssh keys](https://help.github.com/articles/generating-ssh-keys) article for instructions to generate and link an SSH key to your account.

Finally, send your GitHub and Bitbucket username to Maurice, Vlad, or another team member so you can be granted access.

## Getting Started

### Install ROS Indigo
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full ros-indigo-moveit-full
```
If you are short on space, you may want to try to install ``ros-indigo-desktop`` instead.

### Install Matlab, MOSEK, and Gurobi
To install the required dependencies on a fresh installation of Ubuntu 14.04 LTS, please follow the [DRC instructions to install Matlab](https://github.com/mitdrc/drc#install-matlab), [MOSEK](https://github.com/mitdrc/drc#instructions-for-mosek), and [Gurobi](https://github.com/mitdrc/drc#instructions-for-gurobi).

### Using the provided scripts for automatic setup

To automatically check out this repository, you can run:
```
curl https://raw.githubusercontent.com/ipab-slmc/ipab-distro/cb88b145b1421b5c90871c73373f36474a6eed0d/bootstrap.sh?token=ABll_D_yixrpwrlpxI9UtFxfT7amBIt5ks5WE6JOwA%3D%3D | bash
```

This will download and run the ``bootstrap.sh`` script which clones this repository and sets up the sandbox remotes for subrepositories, Matlab startup paths, as well as some ``.bashrc`` shortcuts.

Following this, other dependencies (apt-get based installs as well as LCM) can be easily installed by running the provided script:
```
cd ~/ipab-distro
./install_dependencies.sh
```

After this, the whole system can be built by running:
```
./build.sh
```

### Helpful Aliases
For your ``.bashrc`` file:
```
source /opt/ros/indigo/setup.bash
```

and helpful aliases (can also be in ``.bashrc`` or in bash_aliases):
```
alias catkin_make_reldeb='catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo'
alias catkin_make_rel='catkin_make -DCMAKE_BUILD_TYPE=Release'
alias init_drc='source ~/ipab-distro/drc/software/config/drc_environment.sh'
```

### Manual setup

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
    source $DRC_BASE/../ipab-ros-workspace/devel/setup.bash

The devel/setup.bash file will be created after you compile the catkin workspace. You may want to source this file after the first compilation.

Install the required ROS packages:

    ros-indigo-tf ros-indigo-image-transport ros-indigo-tf-conversions ros-indigo-image-geometry ros-indigo-cv-bridge ros-indigo-std-srvs

The catkin workspace is located in ipab-ros-workspace. The workspace is compiled separately by running

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

