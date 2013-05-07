set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

# Create a shared library liblaser-utils.so
add_library(drc-utils SHARED
    Clock.cpp
    LcmWrapper.cpp
    BotWrapper.cpp
)

# make the header public
# install it to include/drc_utils
pods_install_headers(
  Clock.hpp
  LcmWrapper.hpp
  BotWrapper.hpp
  PointConvert.h
  DESTINATION drc_utils)

# make the library public
pods_install_libraries(drc-utils)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES bot2-core lcm eigen3 bot2-frames)
pods_use_pkg_config_packages(drc-utils ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(drc-utils
    CFLAGS
    LIBS -ldrc-utils
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
