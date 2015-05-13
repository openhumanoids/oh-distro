add_executable(drc-contact-monitor contact-monitor.cpp)
target_link_libraries(drc-contact-monitor lcm)
pods_install_executables(drc-contact-monitor)