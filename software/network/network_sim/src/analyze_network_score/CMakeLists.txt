add_executable (drc-analyze-network-score analyze-network-score.cpp )
target_link_libraries(drc-analyze-network-score lcm goby_common)
pods_install_executables(drc-analyze-network-score)
