#
# ImageCalUtility - Makefile
#

#
# Include all of our child directories.
#

include_directories (
                      ${BASE_DIRECTORY}/source
                      ${OpenCV_INCLUDE_DIRES}
                    )
#
# Setup the executable that we will use.
#

add_executable(ImageCalUtility ImageCalUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries(ImageCalUtility MultiSense
                                      pthread
                                      rt
                                      ${OpenCV_LIBS})
