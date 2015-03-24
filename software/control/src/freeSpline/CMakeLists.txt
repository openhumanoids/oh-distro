# add_library(cvxgen_freeSpline SHARED solver.c matrix_support.c ldl.c util.c csolve.c)

# if(WIN32 AND NOT CYGWIN)
# 	set_target_properties(cvxgen_freeSpline PROPERTIES SUFFIX ".pyd")
# else()
# 	set_target_properties(cvxgen_freeSpline PROPERTIES PREFIX "" SUFFIX ".so")
# endif()

# set(mex_suffix mexa64)
# if(APPLE)
# 	set(mex_suffix mexmaci64)
# endif()

# add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/freeSplineMex.${mex_suffix}
#   COMMAND matlab -nosplash -nodesktop -r \"make_csolve\; quit\(\)\;\"
#   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# add_custom_target(freeSplineMex ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/csolve.mexa64);

# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/csolve.${mex_suffix} DESTINATION

# add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/freeSpline.c
#                    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/solver.c ${CMAKE_CURRENT_SOURCE_DIR}/freeSpline.c)
# add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/freeSpline.h
#                    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/solver.h ${CMAKE_CURRENT_SOURCE_DIR}/freeSpline.h)
# add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/freeSplineMex.c
#                    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/csolve.c ${CMAKE_CURRENT_SOURCE_DIR}/freeSplineMex.c)

# add_custom_target(rename ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/freeSpline.h ${CMAKE_CURRENT_SOURCE_DIR}/freeSpline.c ${CMAKE_CURRENT_SOURCE_DIR}/freeSplineMex.c)


add_mex(freeSplineMex csolve.c ldl.c matrix_support.c solver.c util.c)
set_target_properties(freeSplineMex PROPERTIES COMPILE_FLAGS "-Wall -Os")
target_link_libraries(freeSplineMex m)

install(TARGETS freeSplineMex DESTINATION matlab)

add_library(freeSpline solver.c ldl.c matrix_support.c util.c)
pods_install_libraries(freeSpline)
pods_install_headers(solver.h DESTINATION freeSpline)
pods_install_pkg_config_file(freeSpline
  LIBS -lfreeSpline
  REQUIRES
  VERSION 0.0.1)
