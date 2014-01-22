cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME splice-isam-trajectory-into-log)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(mav-mapping-splice-traj-into-log src/splice-into-log.cpp)
pods_use_pkg_config_packages(mav-mapping-splice-traj-into-log lcm bot2-core)
pods_install_executables(mav-mapping-splice-traj-into-log)

add_executable(mav-mapping-correct-pose-roll src/correct-pose-roll.cpp)
pods_use_pkg_config_packages(mav-mapping-correct-pose-roll lcm eigen3 eigen-utils bot2-frames bot2-param-client
	lcmgl-utils laser-util)
pods_install_executables(mav-mapping-correct-pose-roll)

add_executable(mav-mapping-publish-projected-stats src/publish-projected-stats.cpp)
pods_use_pkg_config_packages(mav-mapping-publish-projected-stats 
	eigen3 eigen-utils bot2-frames laser-util octomap-util gpf-laser-lib path-util)
pods_install_executables(mav-mapping-publish-projected-stats)
