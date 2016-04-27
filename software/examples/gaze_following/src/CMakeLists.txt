# Demo application to do gaze tracking with the Valkyrie head
add_executable(oh-gaze-tracking-demo gazeTrackingDemo.cpp)
target_link_libraries(oh-gaze-tracking-demo boost_thread boost_system boost_filesystem boost_signals)
pods_use_pkg_config_packages(oh-gaze-tracking-demo drake-rbm drake-ik lcm bot2-core bot2-frames eigen3 model-client)
pods_install_executables(oh-gaze-tracking-demo )
