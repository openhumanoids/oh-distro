cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

#include(cmake/pods.cmake)
set(POD_NAME valkyrie_translator)

project(valkyrie_translator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pluginlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp std_msgs ros_control controller_interface
  DEPENDS system_lib pluginlib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

######################################################
add_library(LCM2ROSControl src/LCM2ROSControl.cpp)
target_link_libraries(LCM2ROSControl ${catkin_LIBRARIES} )
pods_use_pkg_config_packages(LCM2ROSControl lcm lcmtypes_pronto-lcmtypes)
