#
# TestSensorPod - Makefile
#

#
# Include all of our child directories.
#

include_directories (
                      ${BASE_DIRECTORY}/source
                      ${BASE_DIRECTORY}/sharedTech/utilities
                    )
#
# Setup the executable that we will use.
#

add_executable(TestSensorPod TestSensorPod.cc)

#
# Specify libraries against which to link.
#

target_link_libraries(TestSensorPod SensorPodCommunications
                                    Platform
                                    pthread
                                    rt)
