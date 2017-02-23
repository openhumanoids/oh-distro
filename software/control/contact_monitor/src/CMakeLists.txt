add_executable(drc-contact-monitor contact-monitor.cpp)
target_link_libraries(drc-contact-monitor lcm)
target_include_directories(drc-contact-monitor PRIVATE ${INCLUDE_INSTALL_PATH})
pods_install_executables(drc-contact-monitor)