add_subdirectory(sandia_hand_description)

macro(chain_build pkg_name)
  message("configure chaining for ${pkg_name}")
  if (${ARGC} GREATER 1)
    message("  depends on ${ARGV1}")
    ExternalProject_Add(${pkg_name}
                        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${pkg_name}
                        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/${pkg_name}
                        CMAKE_ARGS
                          -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                          -DSANDIA_HAND_VERSION:STRING=${SANDIA_HAND_VERSION}
                          ${CATKIN_BUILD_BINARY_PACKAGE_STRING}
                        INSTALL_COMMAND ""
                        DEPENDS ${ARGV1} ${ARGV2} ${ARGV3} ${ARGV4} ${ARGV5})
  else()
    ExternalProject_Add(${pkg_name}
                        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${pkg_name}
                        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/${pkg_name}
                        CMAKE_ARGS
                          -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                          -DSANDIA_HAND_VERSION:STRING=${SANDIA_HAND_VERSION}
                          ${CATKIN_BUILD_BINARY_PACKAGE_STRING}
                        INSTALL_COMMAND "")
  endif()
  install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/${pkg_name} install)")
endmacro(chain_build)

chain_build(sandia_hand_msgs)
chain_build(sandia_hand_driver sandia_hand_msgs)
chain_build(sandia_hand_teleop sandia_hand_driver)
