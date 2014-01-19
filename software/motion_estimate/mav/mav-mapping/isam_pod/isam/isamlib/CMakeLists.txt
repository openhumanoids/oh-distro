cmake_minimum_required(VERSION 2.6)

# find all source and header files
file(GLOB SRCS RELATIVE "${PROJECT_SOURCE_DIR}/isamlib" "*.cpp")
file(GLOB INCS RELATIVE "${PROJECT_SOURCE_DIR}/include/isam" "*.h")

# isamlib
add_library(isamlib ${SRCS} ${INCS})
set_property(TARGET isamlib PROPERTY OUTPUT_NAME isam)

# find CHOLMOD as part of suitesparse (on Mac in ufsparse/ instead of suitesparse/)
find_path(CHOLMOD_INCLUDE "cholmod.h" PATH_SUFFIXES suitesparse ufsparse
  DOC "CHOLMOD header file (part of SuiteSparse or independent)")
if(CHOLMOD_INCLUDE MATCHES "NOTFOUND")
  message(FATAL_ERROR "SuiteSparse/CHOLMOD not found, please see the installation instructions for iSAM.")  
endif()
find_library(CXSPARSE_LIBRARY cxsparse DOC "CXSPARSE library location")
find_library(CHOLMOD_LIBRARY cholmod DOC "CHOLMOD library location")
if(CHOLMOD_LIBRARY MATCHES "NOTFOUND")
  message(FATAL_ERROR "SuiteSparse/CHOLMOD not found, please see the installation instructions for iSAM.")  
endif()
# if static library (Macports), have to manually include a number of other libraries
get_filename_component(CHOLMOD_LIB_EXT ${CHOLMOD_LIBRARY} EXT)
if (CHOLMOD_LIB_EXT STREQUAL ".a")
  # static libraries are cumbersome:
  # on some systems one library has to be added, on other systems
  # another one - we'll simply try each of them and add it if available...
  find_library(BLAS_LIBRARY blas)
  if(BLAS_LIBRARY)
    target_link_libraries(isamlib ${BLAS_LIBRARY})
  endif()
  find_library(ATLAS_LIBRARY atlas)
  if(ATLAS_LIBRARY)
    target_link_libraries(isamlib ${ATLAS_LIBRARY})
  endif()
  find_library(LAPACK_LIBRARY lapack)
  if(LAPACK_LIBRARY)
    target_link_libraries(isamlib ${LAPACK_LIBRARY})
  endif()
  find_library(METIS_LIBRARY metis)
  if(METIS_LIBRARY)
    target_link_libraries(isamlib ${METIS_LIBRARY})
  endif()
  get_filename_component(CHOLMOD_LIB_PATH ${CHOLMOD_LIBRARY} PATH)
  target_link_libraries(isamlib -L${CHOLMOD_LIB_PATH} amd camd colamd ccolamd cxsparse cholmod)
else()
  # dynamic library is easy
  target_link_libraries(isamlib ${CXSPARSE_LIBRARY} ${CHOLMOD_LIBRARY})
endif()

add_definitions(-I${CHOLMOD_INCLUDE})

# install library
install(TARGETS isamlib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
# install header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/isam/ DESTINATION include/isam
          FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
