#target_link_libraries(boost boost_signal)

#find_package(Boost COMPONENTS signals REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})

add_executable(gui main.cc)
add_executable(affmain affmain.cc)

#target_link_libraries(simple 
#  ${Boost_SIGNALS_LIBRARY}
#)

set(REQUIRED_PACKAGES_GUI lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames urdf orocos-kdl kdl-parser forward-kinematics otdf_dom)
pods_use_pkg_config_packages(gui gui_widgets ${REQUIRED_PACKAGES_GUI})
pods_use_pkg_config_packages(affmain gui_widgets ${REQUIRED_PACKAGES_GUI})

pods_install_executables(gui affmain)
