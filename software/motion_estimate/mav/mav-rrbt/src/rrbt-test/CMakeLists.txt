# Create an executable program hello-test
add_executable(rrbt-test rrbt-test.cpp)

# use packages 
pods_use_pkg_config_packages(rrbt-test rrbt mav-planning-utils)

# make executable public
pods_install_executables(rrbt-test)

