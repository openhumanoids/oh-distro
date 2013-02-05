Example program for exercising the Carnegie Robotics MultiSense-SL
sensor assembly.

To build on ubuntu:

  > sudo apt-get install libboost-thread-dev
  > cd build && cmake .. && make

To exercise the MultiSense-SL:

  > sudo ./Scripts/configureNetwork.sh
  > cd bin && ./TestSensorPod

Please contact dlr@carnegierobotics.com for all issues related to this
example code.
