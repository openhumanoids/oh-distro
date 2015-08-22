cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME systemtests)
include(cmake/pods.cmake)

# Unit Testing
include(CTest)
set(test_data_dir ${CMAKE_SOURCE_DIR}/../../drc-testing-data)  # using ~/ probably won't work, instead define it relative to the project dir
add_test(NAME test_testLaunchSCS
               COMMAND ${CMAKE_SOURCE_DIR}/src/launchSCS/test.bash) 
