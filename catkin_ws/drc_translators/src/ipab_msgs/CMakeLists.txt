cmake_minimum_required(VERSION 2.8.3)

project(ipab_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  roscpp
  rospy  std_msgs  message_generation)


## Generate messages in the 'msg' folder
add_message_files(   FILES   PlannerRequest.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(  DEPENDENCIES  std_msgs )

catkin_package( 
#  INCLUDE_DIRS include
 CATKIN_DEPENDS std_msgs message_runtime )

include_directories(${catkin_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
