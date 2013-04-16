set(cpp_files 
  opencv_utils.cpp
  math_utils.cpp
  imshow_utils.cpp
  color_utils.cpp
  imgproc_utils.cpp
  plot_utils.cpp
  calib_utils.cpp
  print_utils.cpp
)


set(hpp_files 
  opencv_utils.hpp
  math_utils.hpp
  imshow_utils.hpp
  color_utils.hpp
  imgproc_utils.hpp
  plot_utils.hpp
  calib_utils.hpp
  print_utils.hpp
)  

# Create a shared library libhello.so with a single source file
add_library(perception-opencv-utils SHARED
    ${cpp_files})

set_target_properties(perception-opencv-utils PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(perception-opencv-utils 
    bot2-core opencv)

target_link_libraries(perception-opencv-utils
    pthread)


# make the header public
pods_install_libraries(perception-opencv-utils)
pods_install_headers(${hpp_files} DESTINATION perception_opencv_utils)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES bot2-core opencv)
#pods_use_pkg_config_packages(hello ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(perception-opencv-utils
    CFLAGS
    LIBS -lperception-opencv-utils -lpthread
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
