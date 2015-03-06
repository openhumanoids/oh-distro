add_definitions(-Wall -std=gnu99 -msse2 -msse3 -DHAVE_INTEL)

set(CMAKE_CXX_FLAGS "-g")

#include_directories(/opt/ros/electric/stacks/openni_kinect/openni_camera/include /usr/include/openni)
include_directories(. openni_camera 
  ../../openni/openni/build/openni/Include
  ../../openni/ps_engine/build/ps_engine/Include
  ../../openni/nite/build/Nite-1.3.1.5/Include
  openni
  /usr/include/openni)
#link_directories(/opt/ros/electric/stacks/openni_kinect/openni_camera/lib)

add_executable(kinect-openni-lcm 
  main.cpp 
   pixels.c     
    pixels_sse2.c
    pixels_sse3.c       
    cpuid.c             
    jpeg-utils-ijg.c    
  kinectOpenniLCM.cpp 
  openni_camera/openni_driver.cpp
  openni_camera/openni_device.cpp
  openni_camera/openni_exception.cpp
  openni_camera/openni_device_primesense.cpp
  openni_camera/openni_device_kinect.cpp
  openni_camera/openni_device_xtion.cpp
  openni_camera/openni_device_oni.cpp
  openni_camera/openni_image_yuv_422.cpp
  openni_camera/openni_image_bayer_grbg.cpp
  openni_camera/openni_image_rgb24.cpp
  openni_camera/openni_ir_image.cpp
  openni_camera/openni_depth_image.cpp
)
#/home/spillai/envoy_latest/software/build/lib/
#  
#  /home/spillai/envoy_latest/software/drivers/kinect/openni/nite/build/Nite-1.3.1.5/Bin
#  /home/spillai/envoy_latest/software/drivers/kinect/openni/ps_engine/build/ps_engine/Platform/Linux-x86/Bin/Release
target_link_libraries(kinect-openni-lcm 
  OpenNI
  jpeg
  #  jpeg-utils
  #../../openni/openni/build/openni/Platform/Linux-x86/Redist/Lib/libOpenNI.so 
  z m ${LCMTYPES_LIBS}  boost_program_options boost_thread usb-1.0 boost_system)

pods_use_pkg_config_packages ( kinect-openni-lcm lcm bot2-core glib-2.0)

pods_install_executables(kinect-openni-lcm)

