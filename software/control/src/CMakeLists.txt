

add_mex(RobotStateMonitor RobotStateMonitor.cpp)
pods_use_pkg_config_packages(RobotStateMonitor eigen3 lcm)

add_mex(AtlasCommandPublisher AtlasCommandPublisher.cpp)
pods_use_pkg_config_packages(AtlasCommandPublisher eigen3 lcm)

install(TARGETS RobotStateMonitor AtlasCommandPublisher DESTINATION matlab)


pods_find_pkg_config(maps)
pods_find_pkg_config(opencv)
if (NOT maps_FOUND OR NOT opencv_FOUND)
  add_mex(QPControllermex QPControllermex.cpp)
  pods_use_pkg_config_packages(QPControllermex eigen3 gurobi drake-qp drake-rbm)
  install(TARGETS QPControllermex DESTINATION matlab)

  add_mex(MomentumControllermex MomentumControllermex.cpp)
  pods_use_pkg_config_packages(MomentumControllermex eigen3 gurobi drake-qp drake-rbm)
  install(TARGETS MomentumControllermex DESTINATION matlab)

  add_mex(ContactDetectmex ContactDetectmex.cpp)
  pods_use_pkg_config_packages(ContactDetectmex eigen3 drake-rbm)
  install(TARGETS ContactDetectmex DESTINATION matlab)

  add_mex(linearIKmex linearIKmex.cpp)
  pods_use_pkg_config_packages(linearIKmex eigen3 gurobi drake-qp drake-rbm)
  install(TARGETS linearIKmex DESTINATION matlab)

else()

  add_mex(HeightMapWrapper HeightMapWrapper.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  pods_use_pkg_config_packages(HeightMapWrapper maps eigen3 lcm opencv gurobi drake-qp drake-rbm)

  add_mex(stepCollocationConstraintsMex stepCollocationConstraintsMex.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  set_target_properties(stepCollocationConstraintsMex PROPERTIES COMPILE_FLAGS "-std=c++0x -DUSE_MAPS")
  pods_use_pkg_config_packages(stepCollocationConstraintsMex maps eigen3 opencv)

  add_mex(BotParamClient BotParamClient.cpp)
  pods_use_pkg_config_packages(BotParamClient lcm drc-utils bot2-param-client)

  add_mex(QPControllermex QPControllermex.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  pods_use_pkg_config_packages(QPControllermex maps eigen3 lcm opencv gurobi drake-qp drake-rbm)
  set_target_properties(QPControllermex PROPERTIES COMPILE_FLAGS "-DUSE_MAPS")

  add_mex(MomentumControllermex MomentumControllermex.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  pods_use_pkg_config_packages(MomentumControllermex maps eigen3 lcm opencv gurobi drake-qp drake-rbm)
  set_target_properties(MomentumControllermex PROPERTIES COMPILE_FLAGS "-DUSE_MAPS")

  add_mex(ContactDetectmex ContactDetectmex.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  pods_use_pkg_config_packages(ContactDetectmex maps eigen3 lcm opencv drake-rbm)
  set_target_properties(ContactDetectmex PROPERTIES COMPILE_FLAGS "-DUSE_MAPS")

  add_mex(linearIKmex linearIKmex.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  pods_use_pkg_config_packages(linearIKmex maps eigen3 lcm opencv gurobi drake-qp drake-rbm)
  set_target_properties(linearIKmex PROPERTIES COMPILE_FLAGS "-DUSE_MAPS")

  add_mex(bot_timestamp_now bot_timestamp_now.cpp)
  pods_use_pkg_config_packages(bot_timestamp_now bot2-core)

  add_mex(bot_param_get_seqno bot_param_get_seqno.cpp)
  pods_use_pkg_config_packages(bot_param_get_seqno bot2-param-client)

  add_mex(bot_param_get_server_id bot_param_get_server_id.cpp)
  pods_use_pkg_config_packages(bot_param_get_server_id bot2-param-client)

  install(TARGETS HeightMapWrapper stepCollocationConstraintsMex QPControllermex MomentumControllermex ContactDetectmex linearIKmex bot_timestamp_now bot_param_get_seqno bot_param_get_server_id DESTINATION matlab)

endif()

pods_find_pkg_config(bullet)
if (bullet_FOUND)
  add_definitions( -DBULLET_COLLISION )
endif()


find_package(Java)
include(UseJava)

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


