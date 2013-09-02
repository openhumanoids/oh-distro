#
# FlashUtility - Makefile
#

#
# Include all of our child directories.
#

include_directories (
                      ${BASE_DIRECTORY}/source
                    )
#
# Setup the executable that we will use.
#

add_executable(FlashUtility FlashUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries(FlashUtility MultiSense
                                   pthread
                                   rt)
