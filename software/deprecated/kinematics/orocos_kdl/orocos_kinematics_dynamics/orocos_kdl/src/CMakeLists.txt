file( GLOB KDL_SRCS [^.]*.cpp )
file( GLOB KDL_HPPS [^.]*.hpp [^.]*.inl)

file( GLOB UTIL_SRCS utilities/[^.]*.cpp utilities/[^.]*.cxx)
file( GLOB UTIL_HPPS utilities/[^.]*.h utilities/[^.]*.hpp)

add_library(orocos-kdl SHARED ${KDL_SRCS} ${UTIL_SRCS})

# make the header public
# install it to include/hello
pods_install_headers(${KDL_HPPS} DESTINATION kdl)
pods_install_headers(${UTIL_HPPS} DESTINATION kdl/utilities)

# make the library public
pods_install_libraries(orocos-kdl)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES eigen3)
pods_use_pkg_config_packages(orocos-kdl ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(orocos-kdl
    CFLAGS
    LIBS -lorocos-kdl
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)


