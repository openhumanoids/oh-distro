include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${LCMTYPES_INCLUDE_DIRS})

#add_executable(test-scan-matcher test_scan_matcher.cpp rotations.c)
#pods_use_pkg_config_packages(test-scan-matcher scanmatch)

add_executable(drc-motion-estimate-scanmatcher scan_matcher.cpp rotations.c)
pods_use_pkg_config_packages(drc-motion-estimate-scanmatcher scanmatch lcm opencv)
pods_install_executables(drc-motion-estimate-scanmatcher)




