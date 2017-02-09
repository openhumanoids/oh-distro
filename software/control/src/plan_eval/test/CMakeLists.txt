# test constructing a WalkingPlan
add_executable(test_generic_plan generic_plan_test.cc)
target_link_libraries(test_generic_plan plan-eval)
#target_link_libraries(c_plan_eval ${CMAKE_THREAD_LIBS_INIT})
pods_install_executables(test_generic_plan)