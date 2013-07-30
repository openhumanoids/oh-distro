

add_mex(RobotStateMonitor RobotStateMonitor.cpp)
set_target_properties(RobotStateMonitor PROPERTIES COMPILE_FLAGS "-std=c++0x")
pods_use_pkg_config_packages(RobotStateMonitor eigen3 lcm)

add_mex(AtlasCommandPublisher AtlasCommandPublisher.cpp)
pods_use_pkg_config_packages(AtlasCommandPublisher eigen3 lcm)

install(TARGETS RobotStateMonitor AtlasCommandPublisher DESTINATION matlab)

pods_find_pkg_config(maps)
pods_find_pkg_config(opencv)
if (NOT maps_FOUND OR NOT opencv_FOUND)
  message("I am in the IF now.")
  add_mex(QPControllermex QPControllermex.cpp)
  pods_use_pkg_config_packages(QPControllermex eigen3 gurobi drake-qp drake-rbm)
  set_target_properties(QPControllermex PROPERTIES COMPILE_FLAGS "-std=c++0x")
  install(TARGETS QPControllermex DESTINATION matlab)

else()

  message("I am in the else now.")
  add_mex(HeightMapWrapper HeightMapWrapper.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  set_target_properties(HeightMapWrapper PROPERTIES COMPILE_FLAGS "-std=c++0x")
  pods_use_pkg_config_packages(HeightMapWrapper maps eigen3 lcm opencv gurobi drake-qp drake-rbm)

  add_mex(QPControllermex QPControllermex.cpp mexmaps/ViewClientWrapper.cpp mexmaps/FillMethods.cpp mexmaps/MapLib.cpp)
  pods_use_pkg_config_packages(QPControllermex maps eigen3 lcm opencv gurobi drake-qp drake-rbm)
  set_target_properties(QPControllermex PROPERTIES COMPILE_FLAGS "-DUSE_MAPS -std=c++0x")

  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_BK})

  add_mex(bot_timestamp_now bot_timestamp_now.cpp)
  pods_use_pkg_config_packages(bot_timestamp_now bot2-core)

  install(TARGETS HeightMapWrapper QPControllermex bot_timestamp_now DESTINATION matlab)

endif()

find_package(Java)
include(UseJava)

execute_process(COMMAND pkg-config --variable=classpath lcm-java OUTPUT_VARIABLE LCM_JAR_FILE)
string(STRIP ${LCM_JAR_FILE} LCM_JAR_FILE)
set( cmake_jar_dir ${CMAKE_INSTALL_PREFIX}/share/java )
set( CMAKE_JAVA_INCLUDE_PATH ${LCM_JAR_FILE}:${cmake_jar_dir}/drake.jar:${cmake_jar_dir}/lcmtypes_drake.jar:${cmake_jar_dir}/lcmtypes_drc_lcmtypes.jar:${cmake_jar_dir}/lcmtypes_bot2-core.jar:${CMAKE_CURRENT_SOURCE_DIR} )

file(GLOB java_src_files "*.java")
add_jar(drc_control ${java_src_files})

install_jar(drc_control share/java)

  
