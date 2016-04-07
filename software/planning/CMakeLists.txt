cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME planning)
include(cmake/pods.cmake)

add_subdirectory(src/util)
add_subdirectory(src/capabilityMap)
add_subdirectory(src/test)
add_subdirectory(src/finalPosePlanner)

# Unit Testing
include(CTest)
add_test(NAME test_FPP COMMAND test-FPP)