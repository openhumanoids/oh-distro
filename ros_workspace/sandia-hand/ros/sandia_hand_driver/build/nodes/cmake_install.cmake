# Install script for directory: /home/sisir/sandia-hand/ros/sandia_hand_driver/nodes

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_node")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_node"
         RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sisir/sandia-hand/ros/sandia_hand_driver/bin/sandia_hand_node")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_node")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_node"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib:/home/sisir/sandia-hand/ros/sandia_hand_driver/lib:"
         NEW_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_node")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_finger_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_finger_node")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_finger_node"
         RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sisir/sandia-hand/ros/sandia_hand_driver/bin/sandia_hand_loose_finger_node")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_finger_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_finger_node")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_finger_node"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib:/home/sisir/sandia-hand/ros/sandia_hand_driver/lib:"
         NEW_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_finger_node")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_palm_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_palm_node")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_palm_node"
         RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sisir/sandia-hand/ros/sandia_hand_driver/bin/sandia_hand_loose_palm_node")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_palm_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_palm_node")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_palm_node"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib:/home/sisir/sandia-hand/ros/sandia_hand_driver/lib:"
         NEW_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/image_common/image_transport/lib:/opt/ros/fuerte/stacks/pluginlib/lib:/opt/ros/fuerte/stacks/image_common/camera_info_manager/lib:/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/sandia_hand_loose_palm_node")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

