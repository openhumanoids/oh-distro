add_library(collections_renderer SHARED collections_renderer.cpp)

pods_use_pkg_config_packages(collections_renderer lcm bot2-vis eigen3)

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(collections_renderer PROPERTIES SOVERSION 1)

pods_install_libraries(collections_renderer)

pods_install_headers(collections_renderer.hpp DESTINATION visualization)

pods_install_pkg_config_file(collections_renderer
    LIBS -lcollections_renderer
    REQUIRES bot2-vis lcm visualization
    VERSION 0.0.1)

