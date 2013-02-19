# Create a shared library
add_library(maps SHARED
  DataBlob.cpp
  Utils.cpp
  PointDataBuffer.cpp
  VoxelGrid.cpp
  BotFramesWrapper.cpp
  SensorDataReceiver.cpp
  LocalMap.cpp
  MapManager.cpp
  MapView.cpp
  LcmTranslator.cpp
  ViewClient.cpp
#  ViewServer.cpp
#  MapWrapper.cpp
#    SpatialQuery.cpp
#    DeltaPublisher.cpp
#    DeltaReceiver.cpp
#    SpatialQueryWrapper.cpp
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
  BotFramesWrapper.hpp
  SensorDataReceiver.hpp
  LocalMap.hpp
  MapManager.hpp
  MapView.hpp
  LcmTranslator.hpp
  ViewClient.hpp
#  ViewServer.hpp
#  MapWrapper.hpp
#  SpatialQuery.hpp
#  DeltaPublisher.hpp
#  DeltaReceiver.hpp
#  SpatialQueryWrapper.hpp
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(maps
    CFLAGS -std=c++0x -Wno-deprecated-declarations
    LIBS -lmaps -lboost_thread -lboost_system
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
