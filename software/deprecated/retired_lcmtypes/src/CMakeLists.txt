set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable ")

##############################################
# driver around the final position of the a plan or the current state:
add_executable (republish-retired-messages republish-retired-messages.cpp)
pods_use_pkg_config_packages(republish-retired-messages lcm 
   lcmtypes_drc_lcmtypes  )
pods_install_executables(republish-retired-messages)
