#
# MultiSense - Makefile
#

#
# Recurse into each of these directories.
#

add_subdirectory(LibMultiSense)
add_subdirectory(FlashUtility)
add_subdirectory(ChangeIpUtility)
add_subdirectory(DeviceInfoUtility)
add_subdirectory(SaveImageUtility)
add_subdirectory(ImuTestUtility)
add_subdirectory(ImuConfigUtility)

find_package(OpenCV)
if (OpenCV_FOUND)
  add_subdirectory(ImageCalUtility)
  add_subdirectory(LidarCalUtility)
endif (OpenCV_FOUND)
