add_executable(affordance_server main.cpp AffordanceServer.cpp
               AffordanceState.cpp AffordancePlusState.cpp )
target_link_libraries(affordance_server  boost_thread  boost_system  )
pods_use_pkg_config_packages(affordance_server eigen3 lcm)
pods_install_executables(affordance_server )

