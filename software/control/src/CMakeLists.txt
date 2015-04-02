
# IMPORTANT NOTE!!
# matlab has it's own boost libraries.  DO NOT let any mex file depend
# on the system boost (directly nor indirectly), or you're asking for trouble.

add_executable(SteeringCommandPublisher SteeringCommandPublisher.cpp)
pods_use_pkg_config_packages(SteeringCommandPublisher lcm)

add_mex(RobotStateMonitor RobotStateMonitor.cpp)
pods_use_pkg_config_packages(RobotStateMonitor eigen3 lcm)

add_mex(AtlasCommandPublisher AtlasCommandPublisher.cpp)
pods_use_pkg_config_packages(AtlasCommandPublisher eigen3 lcm)

add_mex(footstepPlanShiftmex footstepPlanShiftmex.cpp)
pods_use_pkg_config_packages(footstepPlanShiftmex eigen3 drake-control-util drake-rbm drake-util)

install(TARGETS RobotStateMonitor AtlasCommandPublisher footstepPlanShiftmex DESTINATION matlab)

add_mex(mxSerialize mxSerialize.cpp)
add_mex(mxDeserialize mxDeserialize.cpp)

pods_find_pkg_config(mini-maps)
pods_find_pkg_config(opencv)

add_mex(linearIKmex linearIKmex.cpp)
pods_use_pkg_config_packages(linearIKmex eigen3 gurobi drake-qp drake-rbm drake-util)
install(TARGETS linearIKmex DESTINATION matlab)

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

add_mex(RobotStateDriver SHARED RobotStateDriver.cpp)
pods_use_pkg_config_packages(RobotStateDriver drake-control-util)
target_link_libraries(RobotStateDriver drakeGeometryUtil)

add_mex(FootContactDriver SHARED FootContactDriver.cpp)

# add_mex(threadedController SHARED threadedController.cpp)
# pods_use_pkg_config_packages(threadedController drake-qp-common)
# target_link_libraries(threadedController RobotStateDriver AtlasCommandDriver FootContactDriver lcm)

add_mex(threadedControllermex threadedControllermex.cpp)
pods_use_pkg_config_packages(threadedControllermex drake-qp-common drake-util)
target_link_libraries(threadedControllermex RobotStateDriver AtlasCommandDriver FootContactDriver lcm)


install(TARGETS bot_timestamp_now bot_param_get_seqno bot_param_get_server_id mxSerialize mxDeserialize threadedControllermex DESTINATION matlab)
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
set( CMAKE_JAVA_INCLUDE_PATH ${LCM_JAR_FILE}:${cmake_jar_dir}/drake.jar:${cmake_jar_dir}/lcmtypes_drake.jar:${cmake_jar_dir}/lcmtypes_drc_lcmtypes.jar:${cmake_jar_dir}/lcmtypes_bot2-core.jar:${CMAKE_CURRENT_SOURCE_DIR} )

set(java_src_files
  LCMLogParser.java
  AtlasStateCommandLogParser.java
  AffordanceFullStateCoder.java
  AtlasCommandCoder.java
  CandidateGraspPublisher.java
  EndEffectorGoalCoder.java
  ForceTorqueStateCoder.java
  GraspGoalCoder.java
  GraspOptStatusPublisher.java
  GraspSeedOptCoder.java
  GraspStateCoder.java
  JointAnglesCoder.java
  NavGoalCoder.java
  PoseGroundMonitor.java
  RobotBodyCoder.java
  RobotNavGoalListener.java
  RobotPlanPublisher.java
  RobotStateCoder.java
  AtlasStateExtraCoder.java
  IrobotJointCommandCoder.java
  SandiaJointCommandCoder.java
  HandStateListener.java
  LCMGLLineBuffer.java
  ViconBodyPointCoder.java
  )

pods_find_pkg_config(lcmtypes_vicon-drc)
if (lcmtypes_vicon-drc_FOUND)
   set( CMAKE_JAVA_INCLUDE_PATH
   "${CMAKE_JAVA_INCLUDE_PATH}:${cmake_jar_dir}/lcmtypes_vicon-drc.jar"
   )
   set( java_src_files ${java_src_files} ViconBodyPointCoder.java)
endif()

add_jar(drc_control ${java_src_files})

install_jar(drc_control share/java)
