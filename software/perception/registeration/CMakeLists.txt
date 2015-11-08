cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME registeration)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(oh-icp-simple src/oh_icp_simple.cpp)
pods_use_pkg_config_packages(oh-icp-simple eigen3)
target_link_libraries(oh-icp-simple pointmatcher boost_system)
pods_install_executables(oh-icp-simple)
