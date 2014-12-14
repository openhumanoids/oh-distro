
#include_directories(
#  ${PROJECT_SOURCE_DIR}/urdf_interface/include
#  ${PROJECT_SOURCE_DIR}/urdf_parser/include
#)

# Create a shared library liburdf_dom.so with a single source file
add_library(urdf_dom SHARED src/link.cpp src/joint.cpp src/urdf_parser.cpp)

# make the header public
# install it to include/urdf_interface
pods_install_headers(include/urdf_parser/urdf_parser.h DESTINATION urdf_parser)

# make the library public
pods_install_libraries(urdf_dom)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES tinyxml)
pods_use_pkg_config_packages(urdf_dom ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(urdf_dom
    CFLAGS
    LIBS -lurdf_dom
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# Create executable programs

# (Superceded by check_urdf.cpp under urdf pod.)
#add_executable(check_urdf_interface src/check_urdf.cpp)
#pods_use_pkg_config_packages(check_urdf_interface urdf_dom)
#pods_install_executables(check_urdf_interface)


add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
pods_use_pkg_config_packages(urdf_to_graphiz urdf_dom)
pods_install_executables(urdf_to_graphiz)

#add_executable(mem_test test/memtest.cpp)
#pods_use_pkg_config_packages(mem_test urdf_dom)
#pods_install_executables(mem_test)




