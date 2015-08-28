cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME systemtests)
include(cmake/pods.cmake)

# Unit Testing
include(CTest)
set(test_data_dir ${CMAKE_SOURCE_DIR}/../../drc-testing-data)  # using ~/ probably won't work, instead define it relative to the project dir
add_test(NAME test_testLaunchSCS COMMAND ${CMAKE_SOURCE_DIR}/src/launchSCS/test.bash) 
add_test(NAME test_testExoticaLWR COMMAND ${CMAKE_SOURCE_DIR}/src/exoticaLWR/test.bash) 

# Example unit test that launches SCS and Drake Designer:

# add_test(NAME test_testMyTest COMMAND ${CMAKE_SOURCE_DIR}/src/generic_test.bash MyTest config.pmd 100)  

# This entry creates a unit test which launches using the generic_test.bash script.
# 3 arguments must be provided:
# Name of the test - this will be used to name the temporary file used to store the results of python script (see exoticaLWR/test.py)
# Sherif config file - a full path to the *.pmd file (see exoticaLWR/kika_lwr_exotica.pmd)
# Maximum alowed time in seconds

# Notes:
# The *.pmd file has to define a script called start and this script has to block the execution using a wait command on the Drake Designer entry.
# For the python script to end the test reporting success, the script has to end with:
#   with open(os.environ.get('SYSTEMTEST_RESULT_FILE'),'w+') as f:
#     f.write('1\n')
#   exit()
# On failure, the python script can just call exit().
