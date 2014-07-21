project(Planning)
cmake_minimum_required(VERSION 2.8)
find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(Planner Planner.cpp PathPlanner.cpp)
target_link_libraries(Planner ${PCL_LIBRARIES} ${OpenCV_LIBS} )
