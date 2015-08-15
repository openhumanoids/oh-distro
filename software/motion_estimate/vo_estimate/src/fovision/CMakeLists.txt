set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


add_definitions(-Wall )
#add_definitions(-Wall -std=gnu99) # .... compiler warning


add_library(fovision SHARED fovision.cpp )
pods_use_pkg_config_packages(fovision eigen3 libfovis)

target_link_libraries(fovision 
       lcmtypes_fovis-bot2   
       lcmtypes_bot2-core
    boost_thread      
    boost_program_options)

set_target_properties(fovision PROPERTIES SOVERSION 1)
pods_install_libraries(fovision)
pods_install_headers(fovision.hpp     DESTINATION drcvision)
pods_install_pkg_config_file(fovision
    LIBS -lfovision
    REQUIRES 
    VERSION 0.0.1)

    
# Required for FK only:
set(FK_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics model-client)

    

##############################################
add_executable (se-simple-fusion fovision_odometry.cpp)
target_link_libraries(se-simple-fusion boost_system)
pods_use_pkg_config_packages(se-simple-fusion eigen3
  lcm bot2-core bot2-frames   bot2-frames_cpp
  lcmtypes_bot2-core  lcmtypes_fovis-bot2 lcmtypes_microstrain_comm zlib
  voconfig   vofeatures   voestimator   fovision
  pronto_vis  
  opencv image_io_utils
  ${FK_PACKAGES})
pods_install_executables(se-simple-fusion)


##############################################
add_executable (se-simple-vo fovision_simple_odometry.cpp)
target_link_libraries(se-simple-vo boost_system)
pods_use_pkg_config_packages(se-simple-vo eigen3 
  lcm bot2-core bot2-frames   bot2-frames_cpp
  lcmtypes_bot2-core  lcmtypes_fovis-bot2 lcmtypes_microstrain_comm zlib
  voconfig   vofeatures   voestimator   fovision
  pronto_vis  
  opencv image_io_utils
  path-util
  ${FK_PACKAGES})
pods_install_executables(se-simple-vo)
