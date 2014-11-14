# @file CMakeLists.txt 
# 
# Top level build instructions.
#
# Significant history (date, user, job code, action):
#   2012-05-07, dtascione@carnegierobotics.com, RD1034, Created file.

#
# Make sure there is a reasonable version of CMAKE installed.
#

cmake_minimum_required(VERSION 2.8)

# This will be useful for build rules in subordinate CMakeLists files.

set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Define where the result of the build should go.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/bin)

#
# Turn on optimizations and maximum warnings.
#

# Turn on debugging symbols / optimizations and warnings.  To choose
# which build type, use "cmake -DCMAKE_BUILD_TYPE=Debug <directory>" or
# "cmake -DCMAKE_BUILD_TYPE=Release <directory>".

set(CMAKE_C_FLAGS_DEBUG "-g -march=native -Wall -fstrict-aliasing")
set(CMAKE_CXX_FLAGS_DEBUG "-g -march=native -Wall -fstrict-aliasing")

set(CMAKE_C_FLAGS_RELEASE "-g -O2 -Wall -ffunction-sections -fomit-frame-pointer -march=native -mfpmath=both -Wall -fstrict-aliasing")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O2 -Wall -ffunction-sections -fomit-frame-pointer -march=native -mfpmath=both -Wall -fstrict-aliasing")

# Dispatch to subordinate CMakeList.txt files.

add_subdirectory(source)

