set(LIB_NAME "terrain-map")

# Create a shared library
add_library(${LIB_NAME} SHARED
  TerrainMap.cpp
  FillMethods.cpp
)

# make the header public
# install it to include/maps
pods_install_headers(
  TerrainMap.hpp
  DESTINATION ${LIB_NAME}
)

# make the library public
pods_install_libraries(${LIB_NAME})

set(REQUIRED_PACKAGES
  mini-maps
)
pods_use_pkg_config_packages(${LIB_NAME} ${REQUIRED_PACKAGES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(${LIB_NAME}
    CFLAGS -std=c++0x -Wall
    LIBS -l${LIB_NAME}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)


add_executable(terrain-map-test TestMain.cpp)
pods_use_pkg_config_packages(terrain-map-test terrain-map)
pods_install_executables(terrain-map-test)
