# Create a shared library libconstraint.so with a single source file
add_library(constraint_authoring SHARED
            constraint.cc
            constraint_sequence.cc
            constraint_task_space_region.cc)

# make the header public
# install it to include/constraint
pods_install_headers(constraint.h
                      constraint_sequence.h
                      constraint_task_space_region.h 
                      DESTINATION authoring)

# make the library public
pods_install_libraries(constraint_authoring)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES affordance)
pods_use_pkg_config_packages(constraint_authoring ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(constraint_authoring
    CFLAGS
    LIBS -lconstraint_authoring
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
