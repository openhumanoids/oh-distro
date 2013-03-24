add_definitions( -ggdb3 )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(matlab_mex)

# The Matlab.cmake is broken, so we'll just do it ourselves, by golly.
#FIND_PACKAGE(Matlab)
SET(MATLAB_ROOT /usr/local/MATLAB/R2012b)
SET(MATLAB_BIN ${MATLAB_ROOT}/bin/glnxa64)
SET(MATLAB_EXT_LIB ${MATLAB_ROOT}/sys/os/glnxa64)
FIND_LIBRARY(MATLAB_MEX_LIBRARY mex ${MATLAB_BIN} )
FIND_LIBRARY(MATLAB_MX_LIBRARY  mx  ${MATLAB_BIN} )
FIND_LIBRARY(MATLAB_ENG_LIBRARY eng ${MATLAB_BIN} )
FIND_FILE(MATLAB_STDC libstdc++.so.6 ${MATLAB_EXT_LIB} )  # need to link against matlab's version of libstdc++
FIND_PATH(MATLAB_INCLUDE_DIR "mex.h" ${MATLAB_ROOT}"/extern/include" )
SET(MATLAB_LIBRARIES ${MATLAB_MEX_LIBRARY} ${MATLAB_MX_LIBRARY} ${MATLAB_ENG_LIBRARY} ${MATLAB_STDC})
IF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
  SET(MATLAB_FOUND 1)
ENDIF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
MARK_AS_ADVANCED(
  MATLAB_LIBRARIES
  MATLAB_MEX_LIBRARY
  MATLAB_MX_LIBRARY
  MATLAB_ENG_LIBRARY
  MATLAB_INCLUDE_DIR
  MATLAB_FOUND
  MATLAB_BIN
)
INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR})
set(MATLAB_MEXFILE_EXT mexa64)

set(KDL_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics otdf_dom eigen3) 

macro (add_mex project files)
  ADD_LIBRARY(${project} MODULE ${project}.cpp ${mex_files})
  TARGET_LINK_LIBRARIES(${project} ${MATLAB_LIBRARIES}) # "/usr/local/MATLAB/R2012b/sys/os/glnxa64/libstdc++.so.6")
  set_target_properties(${project} PROPERTIES
    PREFIX ""
    SUFFIX  ".${MATLAB_MEXFILE_EXT}"
    )
  pods_install_libraries(${project})
  pods_use_pkg_config_packages(${project} lcmtypes_drc_lcmtypes bot2-param-client ${KDL_PACKAGES})
endmacro(add_mex)

set(mex_files ConstraintApp.cpp)
add_mex(cam_initialize mex_files)
add_mex(cam_destroy mex_files)
add_mex(cam_getCounter mex_files)
add_mex(cam_getObservations mex_files)
add_mex(cam_waitForObservations mex_files)


