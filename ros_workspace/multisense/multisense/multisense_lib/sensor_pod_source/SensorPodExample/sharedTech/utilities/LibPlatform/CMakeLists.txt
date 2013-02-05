# @file LibPlatform/CMakeLists.txt
# 
# Build instructions for CRL SharedTech LibPlatform module.
#
# Significant history (date, user, job code, action):
#   2012-05-07, dlr@carnegierobotics.com, PR1022, Created file.

include_directories(
  ..
)

add_library(
  Platform SHARED 

  Constants.cc
  Constants.hh
  Functional.hh
  ReferenceCount.hh
  StandardException.cc
  StandardException.hh
  TimeStamp.cc
  TimeStamp.hh
  Units.hh
)

# need rt library for clock_gettime

target_link_libraries(
  Platform

  rt
)
