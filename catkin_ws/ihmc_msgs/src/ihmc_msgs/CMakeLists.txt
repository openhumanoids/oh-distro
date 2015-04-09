cmake_minimum_required(VERSION 2.8.3)

project(ihmc_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  roscpp
  rospy  std_msgs  message_generation geometry_msgs)


## Generate messages in the 'msg' folder
add_message_files(   FILES   
    ArmJointTrajectoryPacketMessage.msg
    ChestOrientationPacketMessage.msg
    ComHeightPacketMessage.msg
    FootPosePacketMessage.msg
    FootstepDataListMessage.msg
    FootstepDataMessage.msg
    FootstepStatusMessage.msg
    HandPosePacketMessage.msg
    HeadOrientationPacketMessage.msg
    HighLevelStatePacketMessage.msg
    JointTrajectoryPointMessage.msg
    PauseCommandMessage.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(  DEPENDENCIES  std_msgs geometry_msgs)

catkin_package(  CATKIN_DEPENDS message_runtime )

include_directories(${catkin_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
