cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME affordance)
include(cmake/pods.cmake)

# automatically build LCM types.  
include(cmake/lcmtypes.cmake)
lcmtypes_build()

#---boost 
find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})			   

#--include sub-directories
include_directories(.)
include_directories(./src)
include_directories(./src/affordance_store)


add_library(affordance SHARED 			
			src/AffordanceState.cpp
			src/affordance_store/AffordanceServer.cpp
			src/affordance_store/AffordanceUpWrapper.cpp)		
			  				  	  	
set(REQUIRED_PACKAGES
    lcm
  	lcmtypes_drc_lcmtypes
	)
			  	
			  	
pods_use_pkg_config_packages(affordance ${REQUIRED_PACKAGES})			  	
			  	
target_link_libraries(affordance ${Boost_LIBRARIES})
			  	
			  				  			  	
pods_install_headers(src/AffordanceState.h
					 src/affordance_store/AffordanceServer.h
 					 src/affordance_store/AffordanceUpWrapper.h
					 DESTINATION affordance)

pods_install_libraries(affordance)			  	



pods_install_pkg_config_file(affordance
						     LIBS -laffordance -lboost_thread -lboost_system)

add_executable(affordance_server src/affordance_store/main.cpp )
			   
pods_install_pkg_config_file(affordance_server
 							affordance
						     LIBS -laffordance -lboost_thread -lboost_system)
			   
pods_install_executables(affordance_server)

target_link_libraries(affordance_server affordance ${REQUIRED_PACKAGES})
	
						     