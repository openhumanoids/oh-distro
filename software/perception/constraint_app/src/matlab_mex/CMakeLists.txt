add_definitions( -ggdb3 )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(matlab_mex)

#########################################################################
# The Matlab.cmake is broken, so we'll just do it ourselves, by golly.
#FIND_PACKAGE(Matlab)
SET(MATLAB_ROOT /usr/local/MATLAB/R2012b)
SET(MATLAB_BIN ${MATLAB_ROOT}/bin/glnxa64)
SET(MATLAB_EXT_LIB ${MATLAB_ROOT}/sys/os/glnxa64)
FIND_LIBRARY(MATLAB_MEX_LIBRARY mex ${MATLAB_BIN} )
FIND_LIBRARY(MATLAB_MX_LIBRARY  mx  ${MATLAB_BIN} )
FIND_LIBRARY(MATLAB_ENG_LIBRARY eng ${MATLAB_BIN} )
FIND_FILE(MATLAB_STDC libstdc++.so.6 ${MATLAB_EXT_LIB} )  # need to link against matlab's version of libstdc++
FIND_PATH(MATLAB_INCLUDE_DIR "mex.h" ${MATLAB_ROOT}/extern/include )
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

#########################################################################
FIND_PATH(OTDF_DOM_PATH "otdf_parser.cpp" ~/drc/software/object_template_model/otdf_dom/otdf_parser/src)
message(STATUS ${OTDF_DOM_PATH})

#set(KDL_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics otdf_dom eigen3) 
set(KDL_PACKAGES orocos-kdl kdl-parser forward-kinematics eigen3 tinyxml path-util) 

macro (add_mex project files)
  ADD_LIBRARY(${project} MODULE ${project}.cpp ${mex_files})
  TARGET_LINK_LIBRARIES(${project} ${MATLAB_LIBRARIES})
  set_target_properties(${project} PROPERTIES
    PREFIX ""
    SUFFIX  ".${MATLAB_MEXFILE_EXT}"
    )
  pods_install_libraries(${project})
  pods_use_pkg_config_packages(${project} lcm lcmtypes_drc_lcmtypes bot2-param-client ${KDL_PACKAGES})
endmacro(add_mex)

FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )


#set(mex_files ConstraintApp.cpp ConstraintApp_RB_UKF.cpp)
set(otdf_files otdf_parser/src/expression_parsing.cpp otdf_parser/src/link.cpp otdf_parser/src/joint.cpp otdf_parser/src/otdf_parser.cpp otdf_parser/src/otdf_to_urdf_converter.cpp)
#set(kdl_parser kdl_parser/src/kdl_parser.cpp)
set(mex_files ConstraintApp.cpp ConstraintApp_MB.cpp Affordance.cpp ${otdf_files})# ${kdl_parser})
add_mex(cam_initialize mex_files)
#add_mex(cam_destroy mex_files)
#add_mex(cam_getObservations mex_files)
#add_mex(cam_waitForObservations mex_files)
#add_mex(cam_getResetAndClear mex_files)
#add_mex(cam_getCurrentStateEstimate mex_files)
#add_mex(cam_setCurrentStateEstimate mex_files)
#add_mex(cam_getExpectedObservations mex_files)

#FIND_PACKAGE( Boost 1.44 COMPONENTS program_options REQUIRED )
#INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(dummy dummy.cpp ${mex_files})
target_link_libraries(dummy ${MATLAB_LIBRARIES} /usr/local/MATLAB/R2012b/bin/glnxa64/libboost_thread.so.1.44.0)
pods_use_pkg_config_packages(dummy lcm lcmtypes_drc_lcmtypes bot2-param-client ${KDL_PACKAGES})
pods_install_executables(dummy)