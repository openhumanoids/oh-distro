cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME urdf)
include(cmake/pods.cmake)


# Create a shared library liburdf_dom.so with a single source file
add_library(urdf SHARED src/model.cpp)


# make the header public
# install it to include/urdf_interface
pods_install_headers(include/urdf/model.h DESTINATION urdf)

# make the library public
pods_install_libraries(urdf)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES urdf_dom tinyxml)
pods_use_pkg_config_packages(urdf ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(urdf
    CFLAGS
    LIBS -lurdf
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# Create executable programs
add_executable(check_urdf src/check_urdf.cpp)
pods_use_pkg_config_packages(check_urdf urdf urdf_dom)
pods_install_executables(check_urdf)


