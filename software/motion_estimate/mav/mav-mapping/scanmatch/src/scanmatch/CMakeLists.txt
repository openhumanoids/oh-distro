file(GLOB cpp_files *.cpp)
file(GLOB hpp_files *.hpp)
add_library(scanmatch SHARED ${cpp_files})


target_link_libraries(scanmatch ${IPP_LIBS})

set(REQUIRED_PACKAGES opencv lcmtypes_scanmatch)
pods_use_pkg_config_packages(scanmatch ${REQUIRED_PACKAGES})


# make the shared library public
pods_install_libraries(scanmatch)
#make the headers public
pods_install_headers(${hpp_files} DESTINATION scanmatch)

if (IPP_FOUND)
	set(IPP_LINK_FLAGS -L${IPP_PATH} ${IPP_LIBS})
	message("IPP_LINK_FLAGS: ${IPP_LINK_FLAGS}")
endif()

pods_install_pkg_config_file(scanmatch
    LIBS -lscanmatch ${IPP_LINK_FLAGS} -lpthread
    CFLAGS 
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
