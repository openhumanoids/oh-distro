
# IMPORTANT NOTE!!
# matlab has it's own boost libraries.  DO NOT let any mex file depend
# on the system boost (directly nor indirectly), or you're asking for trouble.

add_executable(SteeringCommandDriver Joystick.cpp SteeringCommandDriver.cpp SteeringCommandPublisher.cpp)
pods_use_pkg_config_packages(SteeringCommandDriver lcm)

add_executable(GamepadDriver GamepadPublisher.cpp Joystick.cpp GamepadDriver.cpp)
pods_use_pkg_config_packages(GamepadDriver lcm)

add_mex(RobotStateMonitor RobotStateMonitor.cpp)
pods_use_pkg_config_packages(RobotStateMonitor eigen3 lcm)

add_mex(AtlasCommandPublisher AtlasCommandPublisher.cpp)
pods_use_pkg_config_packages(AtlasCommandPublisher eigen3 lcm)

add_mex(footstepPlanShiftmex footstepPlanShiftmex.cpp)
pods_use_pkg_config_packages(footstepPlanShiftmex eigen3 drake-control-util drake-rbm drake-mex-util)

install(TARGETS RobotStateMonitor AtlasCommandPublisher footstepPlanShiftmex DESTINATION matlab)

add_mex(mxSerialize mxSerialize.cpp)
add_mex(mxDeserialize mxDeserialize.cpp)

pods_find_pkg_config(mini-maps)
pods_find_pkg_config(opencv)

add_mex(BotParamClient BotParamClient.cpp)
pods_use_pkg_config_packages(BotParamClient lcm drc-utils bot2-param-client)

add_mex(bot_timestamp_now bot_timestamp_now.cpp)
pods_use_pkg_config_packages(bot_timestamp_now bot2-core)

add_mex(bot_param_get_seqno bot_param_get_seqno.cpp)
pods_use_pkg_config_packages(bot_param_get_seqno bot2-param-client)

add_mex(bot_param_get_server_id bot_param_get_server_id.cpp)
pods_use_pkg_config_packages(bot_param_get_server_id bot2-param-client)

add_mex(AtlasCommandDriver SHARED AtlasCommandDriver.cpp)
pods_use_pkg_config_packages(AtlasCommandDriver drake-qp-common)

# NOTE: add_library can NOT be replaced by add_mex here, because executables link against it.
# RobotStateDriver currently does need to include controlUtil.h from drake however, for a typedef.
# It may NOT link against drakeControlUtil however, since that is a mex library, so calling a function defined in that header will result in a linker error
add_library(RobotStateDriver SHARED RobotStateDriver.cpp)
target_link_libraries(RobotStateDriver drakeGeometryUtil)
pods_install_headers(RobotStateDriver.hpp DESTINATION control)

add_mex(FootContactDriver SHARED FootContactDriver.cpp)

add_library(ExponentialForm SHARED ExponentialForm.cpp)
target_link_libraries(ExponentialForm drakePolynomial)
pods_install_libraries(ExponentialForm)
pods_install_headers(ExponentialForm.hpp DESTINATION control)

add_library(QPReactiveRecoveryPlan SHARED QPReactiveRecoveryPlan.cpp)
target_link_libraries(QPReactiveRecoveryPlan iris_cvxgen_ldp drakePolynomial ExponentialForm drakeGeometryUtil drakeTrajectories drakeSplineGeneration drakeRBM lcm drakeLCMUtil)
pods_use_pkg_config_packages(QPReactiveRecoveryPlan drake-convex-hull)
pods_install_libraries(QPReactiveRecoveryPlan)
pods_install_headers(QPReactiveRecoveryPlan.hpp DESTINATION control)
pods_install_pkg_config_file(drc-qp-reactive-recovery
  LIBS -ldrcReactiveRecovery
  CFLAGS -I\${prefix}/include/lcmtypes
  REQUIRES
  VERSION 0.0.1)

add_mex(constructRecoveryMexPointer constructRecoveryMexPointer.cpp)
pods_use_pkg_config_packages(constructRecoveryMexPointer drake-qp-common drake-mex-util drake-control-mex-util)
target_link_libraries(constructRecoveryMexPointer QPReactiveRecoveryPlan)
install(TARGETS constructRecoveryMexPointer DESTINATION matlab)

add_mex(threadedControllermex threadedControllermex.cpp)
pods_use_pkg_config_packages(threadedControllermex drake-qp-common drake-mex-util drake-control-mex-util drake-side)
target_link_libraries(threadedControllermex RobotStateDriver AtlasCommandDriver FootContactDriver lcm)

add_library(AtlasFallDetector SHARED AtlasFallDetector.cpp)
pods_use_pkg_config_packages(AtlasFallDetector lcmtypes_drc_lcmtypes lcm drake-convex-hull)
target_link_libraries(AtlasFallDetector drakeRBM RobotStateDriver)
pods_install_libraries(AtlasFallDetector)
pods_install_headers(AtlasFallDetector.hpp DESTINATION control)

add_executable(drc-atlas-fall-detector runAtlasFallDetector.cpp)
target_link_libraries(drc-atlas-fall-detector AtlasFallDetector)
pods_install_executables(drc-atlas-fall-detector )

add_subdirectory(contact-passthrough)

install(TARGETS bot_timestamp_now bot_param_get_seqno bot_param_get_server_id mxSerialize mxDeserialize threadedControllermex BotParamClient DESTINATION matlab)
install(TARGETS RobotStateDriver AtlasCommandDriver FootContactDriver DESTINATION lib)


if (mini-maps_FOUND AND opencv_FOUND)

  add_mex(HeightMapWrapper HeightMapWrapper.cpp)
  pods_use_pkg_config_packages(HeightMapWrapper terrain-map eigen3 lcm opencv gurobi drake-qp drake-rbm)
  install(TARGETS HeightMapWrapper DESTINATION matlab)

endif()

pods_find_pkg_config(bullet)
if (bullet_FOUND)
  add_definitions( -DBULLET_COLLISION )
endif()


find_package(Java)
include(UseJava)
set(CMAKE_JAVA_COMPILE_FLAGS ${CMAKE_JAVA_COMPILE_FLAGS} -source 6 -target 6)

execute_process(COMMAND pkg-config --variable=classpath lcm-java OUTPUT_VARIABLE LCM_JAR_FILE)
string(STRIP ${LCM_JAR_FILE} LCM_JAR_FILE)
set( cmake_jar_dir ${CMAKE_INSTALL_PREFIX}/share/java )
set( CMAKE_JAVA_INCLUDE_PATH ${LCM_JAR_FILE}:${cmake_jar_dir}/drake.jar:${cmake_jar_dir}/lcmtypes_drake.jar:${cmake_jar_dir}/lcmtypes_atlas.jar:${cmake_jar_dir}/lcmtypes_drc_lcmtypes.jar:${cmake_jar_dir}/lcmtypes_bot2-core.jar:${CMAKE_CURRENT_SOURCE_DIR} )

set(java_src_files
  LCMLogParser.java
  AtlasStateCommandLogParser.java
  AffordanceFullStateCoder.java
  AtlasCommandCoder.java
  ForceTorqueStateCoder.java
  PoseGroundMonitor.java
  RobotPlanPublisher.java
  RobotStateCoder.java
  AtlasStateExtraCoder.java
  HandStateListener.java
  LCMGLLineBuffer.java
  ViconBodyPointCoder.java
  )

pods_find_pkg_config(lcmtypes_vicon)
if (lcmtypes_vicon_FOUND)
   set( CMAKE_JAVA_INCLUDE_PATH
   "${CMAKE_JAVA_INCLUDE_PATH}:${cmake_jar_dir}/lcmtypes_vicon.jar"
   )
   set( java_src_files ${java_src_files} ViconBodyPointCoder.java)
endif()

add_jar(drc_control ${java_src_files})

install_jar(drc_control share/java)

add_subdirectory(tests)
pods_install_python_script(drc-qp-input-snooper python/qp_controller_input_snooper.py)
