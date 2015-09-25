cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

include(cmake/pods.cmake)
set(POD_NAME drc_translators)

project(drc_translators)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ipab_msgs
  ihmc_msgs
  geometry_msgs
  tf
  trajectory_msgs
  sensor_msgs
  image_transport
  std_srvs
  kdl_conversions
  roslint
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp std_msgs ipab_msgs ihmc_msgs tf trajectory_msgs sensor_msgs image_transport geometry_msgs kdl_conversions
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)


add_executable(ros2lcm_ihmc src/ros2lcm_ihmc.cpp)
target_link_libraries(ros2lcm_ihmc ${catkin_LIBRARIES} )
pods_use_pkg_config_packages(ros2lcm_ihmc lcm lcmtypes_pronto-lcmtypes bot2-core eigen3)
add_dependencies(ros2lcm_ihmc ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

add_executable(ros2lcm_camera src/ros2lcm_camera.cpp)
target_link_libraries(ros2lcm_camera ${catkin_LIBRARIES})
pods_use_pkg_config_packages(ros2lcm_camera lcm bot2-core eigen3 opencv)
add_dependencies(ros2lcm_camera ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

add_executable(ros2lcm_stereo src/ros2lcm_stereo.cpp)
target_link_libraries(ros2lcm_stereo ${catkin_LIBRARIES})
pods_use_pkg_config_packages(ros2lcm_stereo lcm bot2-core eigen3 bot2-core eigen3 image_io_utils opencv)
add_dependencies(ros2lcm_stereo ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

add_executable(lcm2ros_ihmc src/lcm2ros_ihmc.cpp)
target_link_libraries(lcm2ros_ihmc ${catkin_LIBRARIES})
pods_use_pkg_config_packages(lcm2ros_ihmc lcm lcmtypes_pronto-lcmtypes bot2-core pronto_vis)
add_dependencies(lcm2ros_ihmc ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)

add_executable(ros2lcm_test src/ros2lcm_test.cpp)
target_link_libraries(ros2lcm_test ${catkin_LIBRARIES})
add_dependencies(ros2lcm_test ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_test lcm lcmtypes_pronto-lcmtypes bot2-core)
######################################################
add_executable(lcm2ros_kuka src/lcm2ros_kuka.cpp)
target_link_libraries(lcm2ros_kuka ${catkin_LIBRARIES})
add_dependencies(lcm2ros_kuka ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp ipab_lwr_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(lcm2ros_kuka lcm lcmtypes_pronto-lcmtypes bot2-core)

add_executable(ros2lcm_kuka src/ros2lcm_kuka.cpp)
target_link_libraries(ros2lcm_kuka ${catkin_LIBRARIES})
add_dependencies(ros2lcm_kuka ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp ipab_lwr_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_kuka lcm lcmtypes_pronto-lcmtypes bot2-core eigen3)

add_executable(ros2lcm_exotica src/ros2lcm_exotica.cpp)
target_link_libraries(ros2lcm_exotica ${catkin_LIBRARIES})
add_dependencies(ros2lcm_exotica ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_exotica lcm lcmtypes_pronto-lcmtypes bot2-core eigen3)

add_executable(lcm2ros_exotica src/lcm2ros_exotica.cpp)
target_link_libraries(lcm2ros_exotica ${catkin_LIBRARIES})
add_dependencies(lcm2ros_exotica ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(lcm2ros_exotica lcm lcmtypes_pronto-lcmtypes bot2-core eigen3)

add_executable(ros2lcm_sdh src/ros2lcm_sdh.cpp)
target_link_libraries(ros2lcm_sdh ${catkin_LIBRARIES})
add_dependencies(ros2lcm_sdh ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_sdh lcm lcmtypes_pronto-lcmtypes bot2-core eigen3)

add_executable(lcm2ros_sdh src/lcm2ros_sdh.cpp)
target_link_libraries(lcm2ros_sdh ${catkin_LIBRARIES})
pods_use_pkg_config_packages(lcm2ros_sdh lcm lcmtypes_pronto-lcmtypes bot2-core std_srvs)

add_executable(ros2lcm_kinect src/ros2lcm_kinect.cpp)
target_link_libraries(ros2lcm_kinect ${catkin_LIBRARIES} z)
add_dependencies(ros2lcm_kinect ipab_msgs_generate_messages_cpp ihmc_msgs_generate_messages_cpp)
pods_use_pkg_config_packages(ros2lcm_kinect lcm lcmtypes_pronto-lcmtypes bot2-core eigen3 image_io_utils opencv)

# add_executable(lcm2ros_botframes src/lcm2ros_botframes.cpp)
# target_link_libraries(lcm2ros_botframes ${catkin_LIBRARIES})
# pods_use_pkg_config_packages(lcm2ros_botframes lcm bot2-core)

roslint_cpp()