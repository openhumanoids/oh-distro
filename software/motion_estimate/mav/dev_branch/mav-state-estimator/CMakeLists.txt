cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mav_estimator)
include(cmake/pods.cmake)
include(cmake/lcmtypes.cmake)
include(cmake/matlab_pods.cmake)
lcmtypes_build()
pods_install_matlab_path(mav_state_est)

add_subdirectory(src)
add_subdirectory(python)

