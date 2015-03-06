include(../../cmake/sse.cmake)
if (NOT SSE3_FOUND)
	message(STATUS, "sse3 not found, not building kinect-lcm")
	return()
endif()
add_definitions(-Wall -std=gnu99 -msse2 -msse3 -DHAVE_INTEL)

add_executable(kinect-lcm main.c 
    timestamp.c 
    pixels.c
    pixels_sse2.c
    pixels_sse3.c
    cpuid.c
    jpeg-utils-ijg.c)

add_executable(test-sender test_sender.c timestamp.c)

target_link_libraries(kinect-lcm 
    #glib-2.0
    gthread-2.0
    freenect
    jpeg
    z
    m
    ${LCMTYPES_LIBS})

target_link_libraries(test-sender
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(kinect-lcm
    glib-2.0
    libusb-1.0
    lcm
    #    jpeg-utils
    )

pods_use_pkg_config_packages(test-sender
    lcm)

pods_install_executables(kinect-lcm)
pods_install_executables(test-sender)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(kinect
    LIBS -lkinect
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)