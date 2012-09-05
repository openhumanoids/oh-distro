
#include_directories(
#  ${PROJECT_SOURCE_DIR}/otdf_interface/include
#  ${PROJECT_SOURCE_DIR}/otdf_parser/include
#)

# Create a shared library liburdf_dom.so with a single source file
add_library(otdf_dom SHARED src/expression_parsing.cpp src/link.cpp src/joint.cpp src/otdf_parser.cpp src/otdf_to_urdf_converter.cpp)
 
# make the header public
# install it to include/urdf_interface
pods_install_headers(include/otdf_parser/otdf_parser.h include/otdf_parser/otdf_urdf_converter.h  DESTINATION otdf_parser)

# make the library public
pods_install_libraries(otdf_dom)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES tinyxml)
pods_use_pkg_config_packages(otdf_dom ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(otdf_dom
    CFLAGS
    LIBS -lotdf_dom
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# Create executable programs

# (Superceded by check_otdf.cpp under urdf pod.)
add_executable(check_otdf_interface src/check_otdf.cpp)
pods_use_pkg_config_packages(check_otdf_interface otdf_dom)
pods_install_executables(check_otdf_interface)


add_executable(otdf_to_graphiz src/otdf_to_graphiz.cpp)
pods_use_pkg_config_packages(otdf_to_graphiz otdf_dom)
pods_install_executables(otdf_to_graphiz)


add_executable(otdf_to_urdf src/otdf_to_urdf.cpp)
pods_use_pkg_config_packages(otdf_to_urdf otdf_dom)
pods_install_executables(otdf_to_urdf)

#add_executable(mem_test test/memtest.cpp)
#pods_use_pkg_config_packages(mem_test otdf_dom)
#pods_install_executables(mem_test)




