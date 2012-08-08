find_package(PkgConfig REQUIRED)
pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building libbot2 renderer")
    return()
endif(NOT BOT2_VIS_FOUND)

#add_definitions(-Wall -std=gnu99)

link_libraries(boost_thread)

add_executable(collections_example collections_example.cpp)
pods_use_pkg_config_packages(collections_example lcm)

# make executables public
pods_install_executables(collections_example)

add_executable(collections_viewer collections_viewer.cpp)
pods_use_pkg_config_packages(collections_viewer bot2-vis 
  bot2-lcmgl-renderer lcm collections_renderer)

pods_install_executables(collections_viewer)
