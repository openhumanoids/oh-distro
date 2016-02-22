add_definitions(-Wall -fPIC -m64 -enable-shared  -enable-pic)##############################################
# -fPIC required for linking to math_utils

add_library(particle SHARED rng.cpp particle.cpp  particle_filter.cpp)
pods_use_pkg_config_packages(particle  eigen3 gsl)
# gsl

target_link_libraries(particle ${LCMTYPES_LIBS} gsl)
#    boost_thread  
#    boost_program_options)
#sandymfallon: math_utils

pods_use_pkg_config_packages(particle pronto_math lcmtypes_bot2-core)

set_target_properties(particle PROPERTIES SOVERSION 1)
pods_install_libraries(particle)
pods_install_headers(rng.hpp particle.hpp particle_filter.hpp
    DESTINATION particle)
pods_install_pkg_config_file(particle
    LIBS -lparticle -lgsl
    REQUIRES 
    VERSION 0.0.1)



 
 
add_executable (particle-filter-example particle_filter_example.cpp )
target_link_libraries(particle-filter-example particle boost_system)
pods_use_pkg_config_packages(particle-filter-example   particle
  gsl  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  pronto_vis pronto_math)
pods_install_executables(particle-filter-example)
