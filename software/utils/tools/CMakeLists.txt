cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME tools)
include(cmake/pods.cmake)


#message(${CMAKE_INSTALL_PREFIX})
file(COPY scripts/drc-svn-stat.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
#pods_install_python_script(drc-svn-stat )
