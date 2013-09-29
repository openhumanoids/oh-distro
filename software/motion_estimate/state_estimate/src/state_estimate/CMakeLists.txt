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

set(required_packages eigen3 drc-lcmtypes kalman-filter inertial-odometry leg-odometry)

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

