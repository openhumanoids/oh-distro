cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME drc_utils)
include(cmake/pods.cmake)

add_subdirectory(src/drc_utils)
add_subdirectory(src/bot_frames_cpp)
#add_subdirectory(src/gtkmm-renderer)
#add_subdirectory(src/joint_utils)
add_subdirectory(src/simple_lcm_bridge)


# require python
find_package(PythonInterp REQUIRED)

# install all python files in the python/ subdirectory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

pods_install_python_script(drc_utils drc_utils.main)

#################################################
pods_install_python_script(drc-spin-lidar python/spin_lidar_command.py)
pods_install_python_script(drc-power-hands python/power_hands.py)
pods_install_python_script(drc-measure-orientation-drift python/measure_orientation_drift.py)
pods_install_python_script(drc-systemwide-tests python/system_test.py)
pods_install_python_script(drc-atlas-pressure-monitor python/atlas_pressure_monitor.py)
pods_install_python_script(drc-controller-contact-passthrough-python python/controller_contact_passthrough.py)
pods_install_python_script(sim-publish-disturbance-and-start-recovery python/publish_disturbance_and_start_recovery.py)
pods_install_python_script(sim-publish-recovery-trigger-off python/publish_recovery_trigger_off.py)
pods_install_python_script(sim-publish-recovery-trigger-on python/publish_recovery_trigger_on.py)
pods_install_python_script(sim-publish-threedof-force python/publish_threedof_force.py)
pods_install_python_script(drc-rolling-log-splicer python/rolling_logs.py)

# Misc Tools added from ipab/tools
pods_install_python_script(drc-send-com-height python/send_com_height.py)
pods_install_python_script(drc-send-pause python/send_pause.py)
pods_install_python_script(drc-send-arm-test python/send_arm_test.py)
pods_install_python_script(drc-send-pose-body python/send_a_pose_msg.py)
pods_install_python_script(drc-plan-passthrough python/plan_passthrough.py)
pods_install_python_script(drc-move-arms-to-set-point python/move_arms_to_set_point.py)
pods_install_python_script(drc-plan-eval-proxi python/plan_eval_proxi.py)
pods_install_python_script(drc-plan-to-scope python/plan_to_scope.py)
pods_install_python_script(drc-republish-old-valkyrie-logs python/republish_old_valkyrie_logs.py)

# not using python interp:
file(COPY python/drc-robot-pose-util DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY scripts/drc-atlas-rsync DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)


pods_install_python_script(drc-send-robot-state python/send_robot_state.py)
pods_install_python_script(drc-send-rotate-and-record python/send_rotate_and_record.py)
