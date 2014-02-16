cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME tools)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/latency)
add_subdirectory(src/frequency)
add_subdirectory(src/printf-cache)
add_subdirectory(src/systemwide-tests)

# require python
find_package(PythonInterp REQUIRED)

# install all python files in the python/ subdirectory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

pods_install_python_script(drc_latency latency.drc_latency)
pods_install_python_script(drc_latency_ros latency.latency_ros)
pods_install_python_script(drc_latency_translator latency.latency_translator)
pods_install_python_script(drc_latency_odometry latency.latency_odometry)


# not using python interp:
file(COPY scripts/drc-svn-stat.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY scripts/drc-lcm-logger.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY scripts/drc-robot-pose-util DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY scripts/drc-webcam-audio-client DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY scripts/drc-webcam-audio-server DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

pods_install_python_script(drc-scope-motionest scripts/python_lcmscope.py)
pods_install_python_script(drc-scope-bandwidth scripts/bandwidth_scope.py)
pods_install_python_script(drc-scope-jointrate scripts/jointrate_scope.py)
pods_install_python_script(drc-set-sim-rate scripts/send_a_rateset.py)
pods_install_python_script(drc-set-hose scripts/send_a_poseset.py)
pods_install_python_script(drc-init-laser scripts/init_laser.py)
pods_install_python_script(drc-projectile-leap scripts/send_a_projectile_leap.py)
pods_install_python_script(drc-power-hands scripts/power_hands.py)
pods_install_python_script(drc-send-a-message scripts/send_a_status_msg.py)
pods_install_python_script(drc_latency scripts/drc_latency.py)

