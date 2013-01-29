cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


include(cmake/pods.cmake)
set(POD_NAME LCM2ROS_translator)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(ActuatorCmds_translator src/ActuatorCmds_translator.cpp)

pods_install_executables(ActuatorCmds_translator)
pods_use_pkg_config_packages(ActuatorCmds_translator lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(sandiaPositionCommandTranslator src/sandiaPositionCommandTranslator.cpp)

pods_install_executables(sandiaPositionCommandTranslator)
pods_use_pkg_config_packages(sandiaPositionCommandTranslator lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(atlasWithHandsPositionCommandTranslator src/atlasWithHandsPositionCommandTranslator.cpp)

pods_install_executables(atlasWithHandsPositionCommandTranslator)
pods_use_pkg_config_packages(atlasWithHandsPositionCommandTranslator lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(syncActuatorCmds_translator src/syncActuatorCmds_translator.cpp)

pods_install_executables(syncActuatorCmds_translator)
pods_use_pkg_config_packages(syncActuatorCmds_translator lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(positionCommandTranslator src/positionCommandTranslator.cpp)

pods_install_executables(positionCommandTranslator)
pods_use_pkg_config_packages(positionCommandTranslator lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(configurationCommandTranslator src/configurationCommandTranslator.cpp)

pods_install_executables(configurationCommandTranslator)
pods_use_pkg_config_packages(configurationCommandTranslator lcm lcmtypes_drc_lcmtypes bot2-core) 
