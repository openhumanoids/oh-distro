set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -fPIC")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lgomp")

include_directories(. ../cvblobs tracker)
link_libraries(cvblobs)

set(PACKAGE_NAME tld)

# Create a shared library
add_library(${PACKAGE_NAME} SHARED
  Clustering.cpp
  DetectionResult.cpp
  DetectorCascade.cpp
  EnsembleClassifier.cpp
  ForegroundDetector.cpp
  MedianFlowTracker.cpp
  NNClassifier.cpp
  TLD.cpp
  TLDUtil.cpp
  VarianceFilter.cpp
  tracker/bb.cpp
  tracker/bb_predict.cpp
  tracker/fbtrack.cpp
  tracker/lk.cpp
  tracker/median.cpp
)

# make the header public and install it
pods_install_headers(
  Clustering.h
  DetectionResult.h
  DetectorCascade.h
  EnsembleClassifier.h
  ForegroundDetector.h
  IntegralImage.h
  MedianFlowTracker.h
  NNClassifier.h
  NormalizedPatch.h
  TLD.h
  TLDUtil.h
  VarianceFilter.h
  DESTINATION ${PACKAGE_NAME})

# make the library public
pods_install_libraries(${PACKAGE_NAME})

set(REQUIRED_PACKAGES glib-2.0 opencv )
pods_use_pkg_config_packages ( ${PACKAGE_NAME} ${REQUIRED_PACKAGES})
# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(${PACKAGE_NAME}
    CFLAGS
    LIBS -l${PACKAGE_NAME}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
# #-------------------------------------------------------------------------------
# #platform specific
# #TODO: Make this configurable
# if(UNIX)
#     SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
# endif(UNIX)

# if(WINDOWS)
#     SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp")
# endif(WINDOWS)

