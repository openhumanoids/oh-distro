file(GLOB cpp_files *.cpp)
file(GLOB hpp_files *.hpp)

add_library(mav-planning-utils SHARED
    ${cpp_files})

set_target_properties(mav-planning-utils PROPERTIES SOVERSION 1)

set(REQUIRED_LIBS glib-2.0 lcm eigen-utils bot2-core bot2-lcmgl-client eigen3 lcmtypes_mav-planning-utils)

# For eigen3 header files to be in the include path
pods_use_pkg_config_packages(mav-planning-utils ${REQUIRED_LIBS})
    
pods_install_headers(${hpp_files} DESTINATION mav_planning_utils)

pods_install_libraries(mav-planning-utils)

pods_install_pkg_config_file(mav-planning-utils
    LIBS -lmav-planning-utils m
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
