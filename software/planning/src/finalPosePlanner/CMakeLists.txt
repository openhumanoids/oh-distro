add_definitions(-ggdb3 -std=c++0x)

# Drake-IK requires either Gurobi or SNOPT to build
find_package(PkgConfig REQUIRED)
pkg_check_modules(drake-ik drake-ik)

if(drake-ik_FOUND)
	add_library(finalPosePlanner  SHARED FinalPosePlanner.cpp)
	pods_use_pkg_config_packages(finalPosePlanner eigen3 drake-rbm drake-ik)
	set_target_properties(finalPosePlanner PROPERTIES SOVERSION 1)
	pods_install_libraries(finalPosePlanner)
	pods_install_headers( FinalPosePlanner.hpp FPPOutput.hpp
	              DESTINATION finalPosePlanner)
	pods_install_pkg_config_file(finalPosePlanner
	  LIBS 
	  REQUIRES eigen3
	  VERSION 0.0.1)
else(drake-ik_FOUND)
	message(WARNING "Drake IK not found - not building FinalPosePlanner (likely Gurobi or Snopt have been missing when building Drake)")
	pods_install_headers(FPPOutput.hpp
	              DESTINATION finalPosePlanner)
endif(drake-ik_FOUND)
