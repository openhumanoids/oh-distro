add_executable(send_message send_message.c)
pods_use_pkg_config_packages(send_message lcm)

add_executable(listener listener.c)
pods_use_pkg_config_packages(listener lcm)

# make exeutables public
pods_install_executables(send_message listener)
