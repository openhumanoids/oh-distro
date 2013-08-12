cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

project (sandia-hand)
include(ExternalProject)
string(REPLACE "-" "_" PROJECT_IDENT ${PROJECT_NAME})
string(TOUPPER ${PROJECT_IDENT} PROJECT_IDENT)

if ("${CATKIN_BUILD_BINARY_PACKAGE}" STREQUAL "1")
  set (CATKIN_BUILD_BINARY_PACKAGE_STRING "-DCATKIN_BUILD_BINARY_PACKAGE=1")
else()
  set (CATKIN_BUILD_BINARY_PACKAGE_STRING "")
endif()

set (SANDIA_HAND_MAJOR_VERSION 5)
set (SANDIA_HAND_MINOR_VERSION 1)
set (SANDIA_HAND_PATCH_VERSION 15)
set (SANDIA_HAND_VERSION ${SANDIA_HAND_MAJOR_VERSION}.${SANDIA_HAND_MINOR_VERSION})
set (SANDIA_HAND_VERSION_FULL ${SANDIA_HAND_MAJOR_VERSION}.${SANDIA_HAND_MINOR_VERSION}.${SANDIA_HAND_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${SANDIA_HAND_VERSION_FULL}")

# Packaging configuration
set (CPACK_PACKAGE_VERSION "${SANDIA_HAND_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${SANDIA_HAND_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${SANDIA_HAND_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${SANDIA_HAND_PATCH_VERSION}")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";/.hg/;.hgignore;.swp$;/build/")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "@PROJECT_NAME@-@SANDIA_HAND_VERSION_FULL@")
include (CPack)

# Infer ROS distro from the user's environment, allowing the user to override
# manually.
if (NOT ROS_DISTRO)
  find_program(ROSVERSION_PATH rosversion)
  if(NOT ROSVERSION_PATH)
    message(FATAL_ERROR "Can't find path to rosversion executable, which is needed to determine which ROS distro you're using.  Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?  You can manually override this check by passing `-DROS_DISTRO=<distro>` to cmake (but that really should not be necessary).")
  endif()
  execute_process(COMMAND ${ROSVERSION_PATH} -d
                  OUTPUT_VARIABLE ROS_DISTRO
                  RESULT_VARIABLE rosversion_error
                  ERROR_VARIABLE rosversion_stderr
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(rosversion_error)
    message(FATAL_ERROR "`rosversion -d` failed. stdout: ${ROS_DISTRO}.  stderr: ${rosversion_stderr}.")
  endif()
endif()
message(STATUS "Building against ROS distro ${ROS_DISTRO}, expected to be installed in /opt/ros/${ROS_DISTRO}")

# rosversion can appear in /usr/bin and will return a valid result even if the
# user hasn't sourced a ROS setup.sh.  So we'll further check that ROS_ROOT is
# set and bail if not.
if("$ENV{ROS_ROOT}" STREQUAL "")
  message(FATAL_ERROR "ROS_ROOT is not set. Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?")
endif()

set (INCLUDE_INSTALL_DIR "include/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/${PROJECT_NAME}") 
set (LIB_INSTALL_DIR "lib" CACHE STRING "Installation directory for libraries (relative to CMAKE_INSTALL_PREFIX)") 
set (BIN_INSTALL_DIR "bin" CACHE STRING "Installation directory for binaries (relative to CMAKE_INSTALL_PREFIX)") 

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "RELEASE" CACHE STRING 
    "Choose the type of build, options are: debug release profile check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

add_subdirectory(ros)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sandia-hand-config.cmake")
configure_file("${conf_file}.in" "${PROJECT_BINARY_DIR}/${conf_file}" @ONLY)
install(FILES "${PROJECT_BINARY_DIR}/${conf_file}" DESTINATION share/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/cmake COMPONENT cmake)
