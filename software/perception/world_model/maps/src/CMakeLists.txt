# Create a shared library
add_library(maps SHARED
  DataBlob.cpp
  Utils.cpp
  PointDataBuffer.cpp
  VoxelGrid.cpp
  DepthImage.cpp
  BotWrapper.cpp
  SensorDataReceiver.cpp
  ViewBase.cpp
  PointCloudView.cpp
  OctreeView.cpp
  DepthImageView.cpp
  VoxelGridView.cpp
  LocalMap.cpp
  MapManager.cpp
  LcmTranslator.cpp
  ViewClient.cpp
  Collector.cpp
  Surfelizer.cpp
#  ViewServer.cpp
)

# make the header public
# install it to include/maps
pods_install_headers(
  DataBlob.hpp
  Types.hpp
  ThreadSafeQueue.hpp
  Utils.hpp
  PointDataBuffer.hpp
  VoxelGrid.hpp
  DepthImage.hpp
  BotWrapper.hpp
  SensorDataReceiver.hpp
  ViewBase.hpp
  PointCloudView.hpp
  OctreeView.hpp
  DepthImageView.hpp
  VoxelGridView.hpp
  LocalMap.hpp
  MapManager.hpp
  LcmTranslator.hpp
  ViewClient.hpp
  Collector.hpp
  Surfelizer.hpp
#  ViewServer.hpp
  DESTINATION maps
)

# make the library public
pods_install_libraries(maps)

set(REQUIRED_PACKAGES
  eigen3
  pcl_common-1.6
  pcl_filters-1.6
  pcl_features-1.6
  pcl_io-1.6
  pcl_surface-1.6
  octomap
  glib-2.0
  bot2-frames
  lcm
  zlib
  drc-utils
)
pods_use_pkg_config_packages(maps ${REQUIRED_PACKAGES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(maps
    CFLAGS -std=c++0x -Wno-deprecated-declarations
    LIBS -lmaps -lboost_thread -lboost_system
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
