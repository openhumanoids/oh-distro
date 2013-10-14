cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME retired_lcmtypes)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build(C_AGGREGATE_HEADER retired_lcmtypes.h)

include_directories(${LCMTYPES_INCLUDE_DIRS})

pods_install_pkg_config_file(retired-lcmtypes
    LIBS -l${LCMTYPES_LIBS}
    REQUIRES lcm
    VERSION 0.0.1)
