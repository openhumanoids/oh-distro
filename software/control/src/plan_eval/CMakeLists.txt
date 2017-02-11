set(PLAN_EVAL_SRC
    plan_eval.cc
    footstep_plan.cc
    zmp_planner.cc
    generic_plan.cc
    manip_plan.cc
    walking_plan.cc
    plan_eval_common.cc
    utils/rate_limiter.cpp)


add_library(plan-eval SHARED ${PLAN_EVAL_SRC})
pods_use_pkg_config_packages(plan-eval drake-qp-common drake-control-util drake-side drake-rbm)
target_link_libraries(plan-eval
    RobotStateDriver
    AtlasCommandDriver
    FootContactDriver
    lcm
    drakeTrajectories
    drakeLCMUtil
    drakePolynomial
    drakeQPLocomotionPlan
    drakeYAMLUtil
    yaml-cpp)
pods_install_libraries(plan-eval)
pods_install_headers(
    footstep_plan.h
    generate_spline.h
    generic_plan.h
    manip_plan.h
    plan_eval.h
    walking_plan.h
    zmp_planner.h
    plan_eval_common.h
    DESTINATION control)


find_package(Threads)

add_executable(c_plan_eval run_plan_eval.cc)
target_link_libraries(c_plan_eval plan-eval)
#target_link_libraries(c_plan_eval ${CMAKE_THREAD_LIBS_INIT})
pods_install_executables(c_plan_eval)

add_executable(sf_test_zmp test_zmp.cc)
target_link_libraries(sf_test_zmp plan-eval)
pods_install_executables(sf_test_zmp)

add_subdirectory(test)

