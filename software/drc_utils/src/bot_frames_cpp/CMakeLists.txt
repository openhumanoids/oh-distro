# Create a shared library liblaser-utils.so
add_library(bot2-frames_cpp SHARED
    bot_frames_cpp.cpp)


# make the header public
# install it to include/drc_utils
pods_install_headers(
  bot_frames_cpp.hpp
  DESTINATION bot_frames_cpp)

# make the library public
pods_install_libraries(bot2-frames_cpp)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES bot2-frames bot2-core eigen3)
pods_use_pkg_config_packages(bot2-frames_cpp ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(bot2-frames_cpp
    CFLAGS
    LIBS -lbot2-frames_cpp
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
