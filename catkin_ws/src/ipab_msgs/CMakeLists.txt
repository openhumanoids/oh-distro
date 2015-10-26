cmake_minimum_required(VERSION 2.8.3)

project(ipab_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy 
  std_msgs
  trajectory_msgs
  message_generation)


## Generate messages in the 'msg' folder
add_message_files(   FILES PlannerResponse.msg PlannerRequest.msg RobotiqCommand.msg PlanStatus.msg)

## Generate service types
add_service_files(
  FILES
  CollisionCheck.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(  DEPENDENCIES  std_msgs trajectory_msgs )

catkin_package( 
#  INCLUDE_DIRS include
 CATKIN_DEPENDS std_msgs trajectory_msgs message_runtime )
