cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME pd_chain_control)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
# include(cmake/lcmtypes.cmake)
# lcmtypes_build()


# Create a shared library 
# add_library(chain-control SHARED src/chain_controller.cpp)

FILE(GLOB PD_CHAIN_CONTROL_HDRS include/pd_chain_control/[^.]*.hpp)
pods_install_headers(${PD_CHAIN_CONTROL_HDRS} DESTINATION pd_chain_control)

# make the library public
# pods_install_libraries(chain-control)

# uncomment these lines to link against another library via pkg-config
# pods_use_pkg_config_packages(chain-control lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
#pods_install_pkg_config_file(chain-control
#    CFLAGS
#    LIBS -lchain-control
#    REQUIRES ${REQUIRED_PACKAGES}
#    VERSION 0.0.1)


