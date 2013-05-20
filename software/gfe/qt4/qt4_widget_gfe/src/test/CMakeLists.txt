find_package(LibXml2)
include_directories(${LIBXML2_INCLUDE_DIR})

find_package(Bullet)
include_directories(${BULLET_INCLUDE_DIRS})

#find_package(orocos-kdl)
#include_directories(${Orocos-KDL_INCLUDE_DIRS})
#link_directories(${Orocos-KDL_LIBRARY_DIRS})

find_package(tmh_math)
include_directories(${TMH_MATH_INCLUDE_DIR})

find_package(tmh_types)
include_directories(${TMH_TYPES_INCLUDE_DIR})

message(TMH_TYPES_LIBRARIES:${TMH_TYPES_LIBRARIES})

find_package(tmh_plan)
include_directories(${TMH_PLAN_INCLUDE_DIR})

# Create an executable program qt4-widget-gfe-control-test
add_executable(qt4-widget-gfe-control-test qt4_widget_gfe_control_test.cc)
add_executable(qt4-widget-gfe-object-test qt4_widget_gfe_object_test.cc)
add_executable(qt4-widget-gfe-plan-test qt4_widget_gfe_plan_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(qt4-widget-gfe-control-test qt4_widget_gfe)
pods_use_pkg_config_packages(qt4-widget-gfe-object-test qt4_widget_gfe)
pods_use_pkg_config_packages(qt4-widget-gfe-plan-test qt4_widget_gfe)

target_link_libraries(qt4-widget-gfe-control-test ${TMH_PLAN_LIBRARIES} ${TMH_TYPES_LIBRARIES} ${TMH_MATH_LIBRARIES} ${BULLET_LIBRARIES} ${LIBXML2_LIBRARIES})
target_link_libraries(qt4-widget-gfe-object-test ${TMH_PLAN_LIBRARIES} ${TMH_TYPES_LIBRARIES} ${TMH_MATH_LIBRARIES} ${BULLET_LIBRARIES} ${LIBXML2_LIBRARIES})
target_link_libraries(qt4-widget-gfe-plan-test ${TMH_PLAN_LIBRARIES} ${TMH_TYPES_LIBRARIES} ${TMH_MATH_LIBRARIES} ${BULLET_LIBRARIES} ${LIBXML2_LIBRARIES})

# make executable public
pods_install_executables(qt4-widget-gfe-control-test)
pods_install_executables(qt4-widget-gfe-object-test)
pods_install_executables(qt4-widget-gfe-plan-test)
