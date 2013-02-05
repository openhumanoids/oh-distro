#
# MultiSense - Makefile
#
# Author:
#     Dan Tascione <mistake@rhqsoftware.com>
#

#
# Include all of our child directories.
#

include_directories (
                      ${BASE_DIRECTORY}/sharedTech/utilities
                      ${BASE_DIRECTORY}/external/include
                    )

#
# Add in all of the source files in this directory.
#

add_library(SensorPodCommunications SHARED AbstractSerializedMessage.h
                                           MessageIdentifiers.h
                                           SensorPodCommunications.cc
                                           SensorPodCommunications.h
                                           SensorPodMessageBuffer.cc
                                           SensorPodMessageBuffer.h
                                           VersionResponseMessage.h)

#
# We want to link against our child libraries.
#

target_link_libraries(SensorPodCommunications Platform)
