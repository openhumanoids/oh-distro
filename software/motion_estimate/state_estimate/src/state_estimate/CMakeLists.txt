set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable -std=c++0x")

set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

set(srcs
  IMUFilter.cpp
  StateEstimator.cpp
  StateEstimateApplication.cpp
  JointFilters.cpp
  StateEstimatorUtilities.cpp
  )

set(required_packages eigen3 drc-lcmtypes kalman-filter inertial-odometry leg-odometry bot2-frames_cpp bot2-frames
                      urdf orocos-kdl kdl-parser forward-kinematics model-client )

#SET(CMAKE_BUILD_TYPE Release)

add_library(StateEstimate  SHARED ${srcs})
target_link_libraries(StateEstimate ${Boost_LIBRARIES})
pods_use_pkg_config_packages(StateEstimate ${required_packages})


add_executable(state_estimate main.cpp)
target_link_libraries(state_estimate StateEstimate)

pods_install_headers(
  IMUBatchProcessor.h
  IMUFilter.h
  IMUMessageProducer.h
  LCMProducers.h
  LCMSubscriber.h
  LCMThread.h
  Macros.h
  QueueTypes.h
  SharedPtrMacros.h
  StateEstimateApplication.h
  StateEstimator.h
  SynchronizedQueue.h
  ThreadLoop.h
  JointFilters.h
  StateEstimatorUtilities.h
  DESTINATION state_estimate)


pods_install_libraries(StateEstimate)
pods_install_executables(state_estimate)

