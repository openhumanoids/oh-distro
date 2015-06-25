cmake_minimum_required(VERSION 2.8.3)

project(atlas_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  roscpp
  rospy  std_msgs geometry_msgs actionlib_msgs message_generation)


## Generate messages in the 'msg' folder
add_message_files(   FILES  
AtlasBehaviorFeedback.msg
AtlasBehaviorManipulateFeedback.msg
AtlasBehaviorManipulateParams.msg
AtlasBehaviorPelvisServoParams.msg
AtlasBehaviorStandFeedback.msg
AtlasBehaviorStandParams.msg
AtlasBehaviorStepData.msg
AtlasBehaviorStepFeedback.msg
AtlasBehaviorStepParams.msg
AtlasBehaviorWalkFeedback.msg
AtlasBehaviorWalkParams.msg
AtlasCommand.msg
AtlasPositionData.msg
AtlasSimInterfaceCommand.msg
AtlasSimInterfaceState.msg
AtlasState.msg
ControllerStatistics.msg
ForceTorqueSensors.msg
SynchronizationStatistics.msg
Test.msg
VRCScore.msg
WalkDemoActionFeedback.msg
WalkDemoActionGoal.msg
WalkDemoAction.msg
WalkDemoActionResult.msg
WalkDemoFeedback.msg
WalkDemoGoal.msg
WalkDemoResult.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(  DEPENDENCIES  std_msgs geometry_msgs actionlib_msgs)

catkin_package(  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs )

