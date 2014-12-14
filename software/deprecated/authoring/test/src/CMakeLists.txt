#boost includes
find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

#=====demo populate executable
add_executable(demo_populate_aff_server demo_populate_affordance_server.cpp)
target_link_libraries(demo_populate_aff_server ${Boost_LIBRARIES} lcm affordance path-util)

# The test program uses the shared library, use the pkg-config file

# make executable public
pods_install_executables(demo_populate_aff_server)
