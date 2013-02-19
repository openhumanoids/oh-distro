cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(network_bridge)

add_executable(drc-network-bridge network-bridge.cpp bridge-handlers.cpp)
target_link_libraries (drc-network-bridge boost_system boost_thread)
pods_use_pkg_config_packages(drc-network-bridge lcm 
             bot2-param-client 
	     lcmtypes_drc_lcmtypes eigen3)
#	    lcmtypes_bot2-core 
pods_install_executables(drc-network-bridge)


include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS shaper-packet.proto)
add_executable(drc-network-shaper network-bridge.cpp shaper-handlers.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries (drc-network-shaper boost_system boost_thread goby_acomms protobuf boost_signals)
pods_use_pkg_config_packages(drc-network-shaper lcm 
             bot2-param-client 
	     lcmtypes_drc_lcmtypes eigen3)
#	    lcmtypes_bot2-core 
pods_install_executables(drc-network-shaper)
