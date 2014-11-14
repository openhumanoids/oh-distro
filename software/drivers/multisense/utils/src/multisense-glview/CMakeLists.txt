add_executable(multisense-glview main.c jpeg-utils-ijg.c)

set(CMAKE_CXX_FLAGS "-g")
add_definitions(-g)

pods_use_pkg_config_packages(multisense-glview lcmtypes_bot2-core lcmtypes_multisense
    lcm
    glib-2.0
    )

target_link_libraries(multisense-glview  lcmtypes_bot2-core lcmtypes_multisense
    ${LCMTYPES_LIBS} 
    z 
    jpeg
    png
    )

if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
else()
  find_package(OpenGL REQUIRED)
  include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
  target_link_libraries(multisense-glview ${OPENGL_LIBRARIES} glut m)
endif()

pods_install_executables(multisense-glview)
