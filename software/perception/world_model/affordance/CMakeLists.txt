cmake_minimum_required(VERSION 2.6.0)

# maximum error checking
if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME affordance)
include(cmake/pods.cmake)

# automatically build LCM types.  
include(cmake/lcmtypes.cmake)
lcmtypes_build()


#========boost and eigen
find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(./src/affordance_store)
include_directories(./src/test)
include_directories(./src)
include_directories(.)

pods_install_headers(
  src/ToString.h
  src/ModelState.h
  src/AffordanceState.h
  src/AffordancePlusState.h
  src/ManipulatorState.h
  src/OpenGL_Affordance.h
  src/OpenGL_Manipulator.h
  src/Collision_Object_Affordance.h
  src/Collision_Object_Manipulator.h
  src/affordance_store/AffordanceServer.h
  src/affordance_store/AffordanceUpWrapper.h
  DESTINATION affordance)

add_library(affordance SHARED 			
  src/AffordanceState.cpp
  src/AffordancePlusState.cpp
  src/ManipulatorState.cpp
  src/OpenGL_Affordance.cpp
  src/OpenGL_Manipulator.cpp
  src/Collision_Object_Affordance.cpp
  src/Collision_Object_Manipulator.cpp
  src/affordance_store/AffordanceServer.cpp	
  src/affordance_store/AffordanceUpWrapper.cpp)
			  				  	  	
pods_install_libraries(affordance)			  	

target_link_libraries(affordance ${Boost_LIBRARIES})			  	

set(REQUIRED_PACKAGES lcm lcmtypes_drc_lcmtypes opengl_object collision_object otdf_dom kinematics_model_gfe)
	  	  	
pods_use_pkg_config_packages(affordance ${REQUIRED_PACKAGES})			  	


pods_install_pkg_config_file(affordance
  LIBS -laffordance
  REQUIRES ${REQUIRED_PACKAGES})			  	

add_executable(affordance_server src/affordance_store/main.cpp )		   
			   
pods_install_executables(affordance_server)

target_link_libraries(affordance_server affordance ${REQUIRED_PACKAGES})


#-----
add_subdirectory(src/utils)
add_subdirectory(src/demo)


#------test
add_executable(server_test src/test/server_test.cpp )		   
target_link_libraries(server_test affordance ${REQUIRED_PACKAGES})
add_executable(affordance_test src/test/affordance_test.cpp )		   
target_link_libraries(affordance_test affordance ${REQUIRED_PACKAGES})
add_executable(lcm_io_test src/test/lcm_io_test.cpp )		   
target_link_libraries(lcm_io_test affordance ${REQUIRED_PACKAGES})


