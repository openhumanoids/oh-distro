#
# MultiSense - Makefile
#

#
# Include all of our child directories.
#

include_directories ( .
                      ${BASE_DIRECTORY}/external/include
                    )

set(MULTISENSE_HEADERS MultiSenseChannel.hh
                       MultiSenseTypes.hh)

set(DETAILS_HEADERS details/channel.hh
                    details/query.hh
                    details/listeners.hh
                    details/signal.hh
                    details/storage.hh)

set(DETAILS_SRC details/channel.cc
                details/public.cc
                details/flash.cc
                details/dispatch.cc
                details/utility/Constants.cc
                details/utility/TimeStamp.cc
                details/utility/Exception.cc)

#
# Add in all of the source files in this directory.
#

add_library(MultiSense SHARED ${MULTISENSE_HEADERS}
                              ${DETAILS_HEADERS}
                              ${DETAILS_SRC})
#
# Versioning...someday lets automate this somehow
#

set_target_properties(MultiSense PROPERTIES VERSION "3.0")

#
# We want to link against our child libraries.
#

target_link_libraries(MultiSense pthread rt)

# create install targets
install(TARGETS MultiSense
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${MULTISENSE_HEADERS} DESTINATION include/MultiSense)
