cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME constraint_app)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/constraint_app)
add_subdirectory(src/matlab_mex)
add_subdirectory(src/test)

enable_testing()
find_program(matlab matlab)
if ( matlab )
   add_test(NAME constraint_app_unit_tests
   		 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		 COMMAND ./testing/test_pf.sh ${matlab})
endif()

#-- Add an Option to toggle the generation of the API documentation
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" ON)
if(BUILD_DOCUMENTATION)
  FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR 
      "Doxygen is needed to build the documentation. Please install it correctly")
  endif()
  #-- Configure the Template Doxyfile for our specific project
#  configure_file(Doxyfile.in 
#                 ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
  #-- Add a custom target to run Doxygen when ever the project is built
  add_custom_target (docs COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/../doc/Doxyfile
			  SOURCES ${PROJECT_BINARY_DIR}/../doc/Doxyfile)
endif()
