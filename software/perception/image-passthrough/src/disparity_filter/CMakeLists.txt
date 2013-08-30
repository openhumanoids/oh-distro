cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


add_executable(test-morphology morphology.cpp)
target_link_libraries(test-morphology )
pods_use_pkg_config_packages(test-morphology opencv )
pods_install_executables(test-morphology)

add_executable(test-morphology-advanced morphology_advanced.cpp)
target_link_libraries(test-morphology-advanced )
pods_use_pkg_config_packages(test-morphology-advanced opencv )
pods_install_executables(test-morphology-advanced)

add_executable(test-morphology-gradient morphological_gradient.cpp)
target_link_libraries(test-morphology-gradient )
pods_use_pkg_config_packages(test-morphology-gradient opencv )
pods_install_executables(test-morphology-gradient)

add_executable(drc-disparity-filter disparity_filter.cpp)
target_link_libraries(drc-disparity-filter )
pods_use_pkg_config_packages(drc-disparity-filter opencv 
        image_io_utils lcm)
pods_install_executables(drc-disparity-filter)
