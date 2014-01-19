set(IPP_FOUND)
set(IPP_LIBS)
set(IPP_HDRS)

message(STATUS "checking for 'IPP'")

foreach(v "6.1" "6.0" "5.3" "5.2" "5.1")
    if(NOT IPP_FOUND)
	    #message(STATUS "Looking for: libippi${CMAKE_SHARED_LIBRARY_SUFFIX}.${v}")
        find_path(IPP_PATH "libippi${CMAKE_SHARED_LIBRARY_SUFFIX}.${v}"
			PATHS ${CMAKE_LIBRARY_PATH} ${CMAKE_SYSTEM_LIBRARY_PATH} /opt/intel/ipp/*/*/sharedlib
            DOC "The path to IPP dynamic libraries")
        if(NOT IPP_PATH)
			#message(STATUS "Looking for: libippiem64t${CMAKE_SHARED_LIBRARY_SUFFIX}.${v}")
            find_path(IPP_PATH "libippiem64t${CMAKE_SHARED_LIBRARY_SUFFIX}.${v}"
            	PATHS ${CMAKE_LIBRARY_PATH} ${CMAKE_SYSTEM_LIBRARY_PATH} /opt/intel/ipp/*/*/sharedlib
                DOC "The path to IPP dynamic libraries")
        endif()
        if(IPP_PATH)
            file(GLOB IPP_HDRS "${IPP_PATH}/../include")
            if(IPP_HDRS)
                set(IPP_FOUND TRUE)
                set(IPP_VERSION ${v})
            endif()
        endif()
    endif()
endforeach()

if (IPP_FOUND)
	message(STATUS "  found IPP version ${IPP_VERSION}")
else()
	message(STATUS "  IPP not found")
endif()

if(IPP_FOUND)
    set(IPP_DEFINES "-DUSE_IPP")
    add_definitions(${IPP_DEFINES})
    include_directories("${IPP_PATH}/../include")
    link_directories("${IPP_PATH}")

    file(GLOB em64t_files "${IPP_PATH}/../lib/*em64t*")
    set(IPP_ARCH)
    if(em64t_files)
        set(IPP_ARCH "em64t")
    endif()

    set(A -l)
    set(B ${IPP_ARCH})
       
    set(IPP_LIBS 
    ${A}ippj${B}  ${A}ippi${B}  ${A}ipps${B} ${A}ippcore${B}
    ${A}ippcc${B} ${A}ippcv${B} ${A}ippm${B} ${A}ippvm${B}
    ${A}ippr${B} ${A}iomp5)
    
    # abuse RPATH
    set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH}:${IPP_PATH})
    
    #message(STATUS "IPP_LIBS: ${IPP_LIBS}")
endif()

