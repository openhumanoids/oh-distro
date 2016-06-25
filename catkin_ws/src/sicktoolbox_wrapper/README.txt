To operate the LMS511 using this driver use the following steps:

1. Checkout and build (including sudo make install)
https://github.com/openhumanoids/sicktoolbox
- installs to /usr/local/include and /usr/local/lib
TODO: make this repo part of externals with a local install into software/build

2. Comment out the application in the CMakeLists.txt: sicklms5xx-app
- build with catkin_make

3a Run ROS driver:
rosrun sicktoolbox_wrapper2 sicklms5xx-app
and republish to LCM:
rosrun sicktoolbox_wrapper2 ros2lcm_sick

3b Run LCM driver:
- If #1 has been done:
  cd software/drivers/sicklms5xx
  make
  sick-lcm    ... publishes scans equivalent to the ROS driver

Initial/Defaults is 1141 points
-	Scan Frequency: 25(Hz)
-	Scan Resolution: 0.1667 (deg)
-	Scan Area: [-5,185]


required packages:
apt-get install build-essential libtool autoconf


mfallon may 2016