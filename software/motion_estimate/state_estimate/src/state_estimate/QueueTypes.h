#ifndef __QueueTypes_h
#define __QueueTypes_h

#include "SynchronizedQueue.h"

#include <lcmtypes/drc/atlas_state_t.hpp>
#include <lcmtypes/drc/atlas_raw_imu_batch_t.hpp>
#include <lcmtypes/drc/robot_state_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>

namespace StateEstimate
{

  typedef SynchronizedQueue<drc::robot_state_t> RobotStateQueue;
  typedef SynchronizedQueue<drc::atlas_state_t> AtlasStateQueue;
  typedef SynchronizedQueue<drc::atlas_raw_imu_t> IMUQueue;
  typedef SynchronizedQueue<drc::atlas_raw_imu_batch_t> IMUBatchQueue;

  typedef SynchronizedQueue<bot_core::pose_t> PoseQueue;

} // end namespace

#endif
