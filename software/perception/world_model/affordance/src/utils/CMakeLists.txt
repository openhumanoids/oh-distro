set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
### needed to read obj:
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

set(PCL_LIBRARIES   pcl_io-1.6)

################ LIBRARIES ###############################
  add_library(affordance-utils  SHARED AffordanceUtils.cpp)
pods_use_pkg_config_packages(affordance-utils ${PCL_LIBRARIES} rgbd_primitives)
target_link_libraries (affordance-utils boost_system )
set_target_properties(affordance-utils PROPERTIES SOVERSION 1)
pods_install_libraries(affordance-utils)
pods_install_headers( AffordanceUtils.hpp DESTINATION affordance)
pods_install_pkg_config_file(affordance-utils
  LIBS -laffordance-utils
  VERSION 0.0.1)
