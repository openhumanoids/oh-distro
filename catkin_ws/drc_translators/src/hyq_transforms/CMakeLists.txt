cmake_minimum_required(VERSION 2.8.3)
project(hyq_transforms)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	tf_conversions
	roscpp
	actionlib
	std_msgs
	actionlib_msgs
	genmsg
	genmsg
)

set(ROS_BUILD_TYPE Release)

catkin_package(CATKIN_DEPENDS roscpp tf tf_conversions roscpp actionlib std_msgs actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(frame_to_base src/FrameToBase.cpp)
target_link_libraries(frame_to_base ${catkin_LIBRARIES})
