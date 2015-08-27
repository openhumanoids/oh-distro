cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME tools)
include(cmake/pods.cmake)

add_subdirectory(src/state_sync_kuka)

pods_install_python_script(ipab-send-com-height python/send_com_height.py)
pods_install_python_script(ipab-send-pause python/send_pause.py)
pods_install_python_script(ipab-send-arm-test python/send_arm_test.py)
pods_install_python_script(ipab-plan-passthrough python/plan_passthrough.py)
pods_install_python_script(ipab-move-arms-to-set-point python/move_arms_to_set_point.py)
pods_install_python_script(ipab-plan-eval-proxi python/plan_eval_proxi.py)
pods_install_python_script(ipab-plan-to-scope python/plan_to_scope.py)
