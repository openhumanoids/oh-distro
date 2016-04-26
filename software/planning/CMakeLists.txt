cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME planning)
include(cmake/pods.cmake)

# Drake-IK requires either Gurobi or SNOPT to build, FPP requires Drake IK
find_package(PkgConfig REQUIRED)
pkg_check_modules(drake-ik drake-ik)

if(drake-ik_FOUND)
	add_subdirectory(src/util)
	add_subdirectory(src/capabilityMap)
	add_subdirectory(src/test)
	add_subdirectory(src/finalPosePlanner)
	add_subdirectory(src/trajectoryTrackingTestAnalysis)

	# Unit Testing
	include(CTest)
	add_test(NAME test_FPP COMMAND test-FPP)
else()
	message(WARNING "Drake IK not found, skipping planning")
endif()
