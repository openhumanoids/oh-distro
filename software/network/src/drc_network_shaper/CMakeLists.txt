cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(network_bridge)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


set(ldpc_sources 
    ldpc/getopt.cpp
    ldpc/ldpc_create_pchk.cpp
    ldpc/ldpc_fec.cpp
    ldpc/ldpc_fec_iterative_decoding.cpp
    ldpc/ldpc_matrix_sparse.cpp
    ldpc/ldpc_rand.cpp
    ldpc/ldpc_scheme.cpp
    ldpc/ldpc_wrapper.cpp
    ldpc/tools.cpp
    )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}
  shaper-packet.proto
  procman-analogs.proto
  robot-state-analogs.proto
  footstep-plan-analogs.proto
  robot-plan-analogs.proto
  ) 
add_executable(drc-network-shaper 
  shaper-app.cpp 
  shaper-handlers.cpp 
  custom-codecs.cpp
  procman-codecs.cpp
  robot-state-codecs.cpp
  footstep-plan-codecs.cpp
  manip-plan-codecs.cpp
  drc_udp_driver.cpp
  ${ldpc_sources}
  ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries (drc-network-shaper boost_system boost_thread boost_date_time goby_acomms goby_common dccl_arithmetic protobuf boost_signals lzma)
pods_use_pkg_config_packages(drc-network-shaper lcm 
             bot2-param-client path-util
	     lcmtypes_drc_lcmtypes eigen3)

pods_install_executables(drc-network-shaper)
