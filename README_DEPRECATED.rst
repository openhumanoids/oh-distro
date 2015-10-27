Neither ROS or Gazebo are currently required. These instructions are likely to be broken
as a result. If ROS is to be supported we will use ROS Indigo.

Gazebo
------

Gazebo (http://gazebosim.org/wiki/2.2/install#Install_Required_Dependencies)

In addition to above packages, run:

::

    sudo apt-get install libboost-all-dev libcegui-mk2-dev libopenal-dev  libswscale-dev libavformat-dev libavcodec-dev libogre-dev libgts-dev libltdl3-dev playerc++ libplayerwkb3.0-dev

DRCSIM requires ROS dependencies listed here: http://gazebosim.org/wiki/DRC/Install#Ubuntu_and_ROS_Groovy

After you have installed ros packages you should run these commands:::

    sudo rosdep init
    rosdep update


Subversion Setup
----------------
Building the DRC Externals requires you to check out copies of several libraries from SVN repositories. You'll need to make sure you've set up your access to those repositories beforehand, or the SVN checkout will fail with a rather obscure error. The easiest way make sure your SVN access is properly set up is to do the following:

::

    svn info https://svn.csail.mit.edu/drc
    svn info https://svn.csail.mit.edu/rrg_pods

Enter your username and password for those repos (which may be different from your CSAIL username/password--ask us if you need access). SVN will remember those credentials for you, although on some systems it will do so by storing them in a plaintext file. Verify that SVN remembers your password by running the commands again and noting that it does not ask you for a username or password. 
