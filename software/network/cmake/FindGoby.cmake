set(GOBY_ROOT_DIR "GOBY_ROOT_DIR-NOTFOUND" CACHE STRING "Path to the root of Goby, e.g. /home/me/goby")

#
# Find include directory
# 
find_path(GOBY_INCLUDE_DIR goby/version.h
  PATHS ${CMAKE_SOURCE_DIR}/../goby ${CMAKE_SOURCE_DIR}/../../goby ${CMAKE_SOURCE_DIR}/../..  ${GOBY_ROOT_DIR}
  PATH_SUFFIXES include)
mark_as_advanced(GOBY_INCLUDE_DIR)

file(READ "${GOBY_INCLUDE_DIR}/goby/version.h" GOBY_VERSION_HPP_CONTENTS)
  
string(REGEX REPLACE ".*#define GOBY_VERSION_MAJOR ([0-9]+).*" "\\1" GOBY_VERSION_MAJOR "${GOBY_VERSION_HPP_CONTENTS}")
string(REGEX REPLACE ".*#define GOBY_VERSION_MINOR ([0-9]+).*" "\\1" GOBY_VERSION_MINOR "${GOBY_VERSION_HPP_CONTENTS}")
string(REGEX REPLACE ".*#define GOBY_VERSION_PATCH ([0-9~a-z]+).*" "\\1" GOBY_VERSION_PATCH "${GOBY_VERSION_HPP_CONTENTS}")


get_filename_component(GOBY_DIR ${GOBY_INCLUDE_DIR}/../ ABSOLUTE)
set(GOBY_LIBRARY_PATH "${GOBY_DIR}/lib")

message("Using Goby in ${GOBY_DIR} (Version ${GOBY_VERSION_MAJOR}.${GOBY_VERSION_MINOR}.${GOBY_VERSION_PATCH})")
#
# Find libraries
# 

function(find_goby_library OUTPUT_VARIABLE LIBRARY_NAME GOBY_DIR)
  find_library(${OUTPUT_VARIABLE} NAMES ${LIBRARY_NAME}
    PATHS ${GOBY_DIR}/lib)

  set(OUTPUT_VARIABLE_CONTENTS ${${OUTPUT_VARIABLE}})
  if(OUTPUT_VARIABLE_CONTENTS)
    #message("OUTPUT_VARIABLE: ${OUTPUT_VARIABLE}")
    #message("OUTPUT_VARIABLE_CONTENTS: ${OUTPUT_VARIABLE_CONTENTS}")
    
    # this forces CMake to explicitly add to the linker the location
    # of the library that we just found
    add_library(${OUTPUT_VARIABLE} SHARED IMPORTED)
    set_target_properties(${OUTPUT_VARIABLE} PROPERTIES IMPORTED_LOCATION ${OUTPUT_VARIABLE_CONTENTS})
    add_library(${LIBRARY_NAME} SHARED IMPORTED)
    set_target_properties(${LIBRARY_NAME} PROPERTIES IMPORTED_LOCATION ${OUTPUT_VARIABLE_CONTENTS})
  endif()
    
  mark_as_advanced(${OUTPUT_VARIABLE})
  set(${OUTPUT_VARIABLE} PARENT_SCOPE)
endfunction()

if(GOBY_VERSION_MAJOR GREATER "1")
  find_goby_library(GOBY_ACOMMS_LIBRARY goby_acomms ${GOBY_DIR})
  find_goby_library(GOBY_COMMON_LIBRARY goby_common ${GOBY_DIR})
#  find_goby_library(GOBY_MOOS_LIBRARY goby_moos ${GOBY_DIR})
  find_goby_library(GOBY_UTIL_LIBRARY goby_util ${GOBY_DIR})

  if(NOT (GOBY_VERSION_MAJOR EQUAL "2" AND GOBY_VERSION_MINOR EQUAL "0"))
    find_package(DCCL)
    if(DEFINED DCCL_INCLUDE_DIR)
      include_directories("${DCCL_INCLUDE_DIR}")
      protobuf_include_dirs("${DCCL_INCLUDE_DIR}")
      endif()
    set(DCCL_LIBRARIES dccl)
  endif()

  #
  # Standard find_package portion
  #

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Goby "Could NOT find Goby. Set GOBY_ROOT_DIR to the path where Goby is on your system (e.g. > cmake -DGOBY_ROOT_DIR=/home/me/goby .)"
    GOBY_DIR 
    GOBY_INCLUDE_DIR
    GOBY_ACOMMS_LIBRARY
#    GOBY_MOOS_LIBRARY
    GOBY_COMMON_LIBRARY
    GOBY_UTIL_LIBRARY
    )

  if(GOBY_FOUND)
    set(GOBY_INCLUDE_DIRS ${GOBY_INCLUDE_DIR})
    set(GOBY_LIBRARIES 
      ${GOBY_ACOMMS_LIBRARY}
#      ${GOBY_MOOS_LIBRARY}
      ${GOBY_COMMON_LIBRARY}
      ${GOBY_UTIL_LIBRARY}
      )
    
    set(GOBY_MOOS_APP_PROTO "goby/moos/protobuf/goby_moos_app.proto")
    set(GOBY_OPTION_EXTENSIONS_PROTO "goby/common/protobuf/option_extensions.proto")
    set(GOBY_MOOS_APP_NAME "GobyMOOSApp")
    set(GOBY_FIELD_DESCRIPTION "(goby.field).description")
    set(GOBY_FIELD_EXAMPLE "(goby.field).example")

    set(GOBY_ROOT_DIR "${GOBY_DIR}" CACHE STRING "Path to the root of Goby, e.g. /home/me/goby" FORCE)
  endif()
endif()

