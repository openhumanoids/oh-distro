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

add_custom_target(MultiSenseUpdater ALL
                  DEPENDS FlashUtility
                  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/_MultiSenseUpdater
                  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/MultiSenseUpdater)

set( MULTISENSE_UPDATER_SED_COMMAND sed -i 's|BINDIR=\"../../bin\"|BINDIR=\"${CMAKE_RUNTIME_OUTPUT_DIRECTORY}\"|' ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/MultiSenseUpdater)

add_custom_target(MultiSenseUpdaterModify ALL
                  DEPENDS MultiSenseUpdater
                  COMMAND ${MULTISENSE_UPDATER_SED_COMMAND} )
