


cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mfallon_sandbox)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()







include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

#rosbuild_add_executable(keyboard_teleop src/keyboard.cpp)
#target_link_libraries(keyboard_teleop boost_thread)









rosbuild_add_executable(keyboard_driving src/keyboard_driving/keyboard_driving.cpp)
target_link_libraries(keyboard_driving  curses boost_thread)


#add_executable (drc-keyboard-driving keyboard_driving.cpp)
#target_link_libraries (drc-keyboard-driving curses    )
pods_use_pkg_config_packages(keyboard_driving
   eigen3  glib-2.0  lcm bot2-core 
    path-util )
#pods_install_executables(keyboard_driving)

#add_subdirectory(src/keyboard_driving)
