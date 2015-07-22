cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME tools)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/frequency)
add_subdirectory(src/printf-cache)
add_subdirectory(src/systemwide-tests)
add_subdirectory(src/contact-passthrough)

# require python
find_package(PythonInterp REQUIRED)

# install all python files in the python/ subdirectory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

# not using python interp:
file(COPY scripts/drc-robot-pose-util DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY scripts/drc-atlas-rsync DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

pods_install_python_script(drc-scope-motionest scripts/python_lcmscope.py)
pods_install_python_script(drc-scope-bandwidth scripts/bandwidth_scope.py)
pods_install_python_script(drc-scope-jointrate scripts/jointrate_scope.py)
pods_install_python_script(drc-spin-lidar scripts/spin_lidar_command.py)
pods_install_python_script(drc-power-hands scripts/power_hands.py)
pods_install_python_script(drc-measure-orientation-drift scripts/measure_orientation_drift.py)

pods_install_python_script(drc-systemwide-tests python/system_test.py)

pods_install_python_script(drc-atlas-pressure-monitor python/atlas_pressure_monitor.py)
pods_install_python_script(drc-controller-contact-passthrough-python python/controller_contact_passthrough.py)

pods_install_python_script(sim-publish-disturbance-and-start-recovery scripts/publish_disturbance_and_start_recovery.py)
pods_install_python_script(sim-publish-recovery-trigger-off scripts/publish_recovery_trigger_off.py)
pods_install_python_script(sim-publish-recovery-trigger-on scripts/publish_recovery_trigger_on.py)
pods_install_python_script(sim-publish-threedof-force scripts/publish_threedof_force.py)
pods_install_python_script(drc-rolling-log-splicer python/rolling_logs.py)
