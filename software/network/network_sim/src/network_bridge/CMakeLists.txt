cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(network_bridge)

add_executable(drc-network-bridge network-bridge.cpp bridge-handlers.cpp)
target_link_libraries (drc-network-bridge boost_system boost_thread)
pods_use_pkg_config_packages(drc-network-bridge lcm 
             bot2-param-client path-util
	     lcmtypes_drc_lcmtypes eigen3)
#	    lcmtypes_bot2-core 
pods_install_executables(drc-network-bridge)


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
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR} shaper-packet.proto procman-analogs.proto) 
add_executable(drc-network-shaper 
  network-bridge.cpp 
  shaper-handlers.cpp 
  custom-codecs.cpp
  drc_udp_driver.cpp
  ldpc/ldpc_wrapper.cpp
  ${ldpc_sources}
  ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries (drc-network-shaper boost_system boost_thread boost_date_time goby_acomms goby_common protobuf boost_signals)
pods_use_pkg_config_packages(drc-network-shaper lcm 
             bot2-param-client path-util
	     lcmtypes_drc_lcmtypes eigen3)
#	    lcmtypes_bot2-core 
pods_install_executables(drc-network-shaper)

add_executable(drc-network-test network-test.cpp)
target_link_libraries (drc-network-test goby_common)
pods_use_pkg_config_packages(drc-network-test lcm 
	     lcmtypes_drc_lcmtypes)
pods_install_executables(drc-network-test)
