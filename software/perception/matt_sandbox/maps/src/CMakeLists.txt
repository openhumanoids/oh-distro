# Create a shared library
add_library(maps SHARED
    MapManager.cpp
    MapChunk.cpp
    PointDataBuffer.cpp
    SpatialQuery.cpp
    WorldModel.cpp
    DenseGrid3d.cpp
    SensorDataReceiver.cpp
)

# make the header public
# install it to include/maps
pods_install_headers(
  MapManager.hpp
  MapChunk.hpp
  PointDataBuffer.hpp
  SpatialQuery.hpp
  WorldModel.hpp
  DenseGrid3d.hpp
  SensorDataReceiver.hpp
  DESTINATION maps
)

# make the library public
pods_install_libraries(maps)

set(REQUIRED_PACKAGES
  eigen3
  pcl_common-1.6
  pcl_octree-1.6
  pcl_filters-1.6
  pcl_features-1.6
  pcl_io-1.6
  glib-2.0
  bot2-frames
)
pods_use_pkg_config_packages(maps ${REQUIRED_PACKAGES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(maps
    CFLAGS -std=c++0x -Wno-deprecated-declarations
    LIBS -lmaps -lboost_thread
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
