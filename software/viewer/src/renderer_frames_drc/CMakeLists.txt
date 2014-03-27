add_definitions(-std=gnu99)
# Create a shared library libbot2-frames-renderer.so with a single source file
add_library(bot2-frames-renderers-drc SHARED
    coord_frames_renderer.c )#articulated_body_renderer.c frame_modifier_renderer.c)
  
set(REQUIRED_LIBS bot2-core bot2-param-client bot2-vis bot2-frames)
  
pods_use_pkg_config_packages(bot2-frames-renderers-drc ${REQUIRED_LIBS})

target_link_libraries(bot2-frames-renderers-drc 
    ${OPENGL_LIBRARIES} 
    ${LIBJPEG_LIBRARIES})

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(bot2-frames-renderers-drc PROPERTIES SOVERSION 1)

# make the header public
# install it to include/bot_frames
pods_install_headers(bot_frames_renderers.h DESTINATION bot_frames_drc)

# make the library public
pods_install_libraries(bot2-frames-renderers-drc)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(bot2-frames-renderers-drc
    CFLAGS
    LIBS -lbot2-frames-renderers-drc -ljpeg
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
    
    
#build the test-viewer
#add_executable(test-viewer test_viewer.c)
#target_link_libraries(test-viewer ${GLUT_LIBRARIES})
#pods_use_pkg_config_packages(test-viewer 
#                                         bot2-vis 
#                                         bot2-lcmgl-renderer 
#                                         bot2-frames-renderers)
