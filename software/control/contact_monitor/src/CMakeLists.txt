add_executable(drc-contact-monitor-cpp contact-monitor.cpp)
target_link_libraries(drc-contact-monitor-cpp lcm)
pods_install_executables(drc-contact-monitor)