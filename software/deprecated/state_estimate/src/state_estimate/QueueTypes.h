#ifndef __QueueTypes_h
#define __QueueTypes_h

#include "SynchronizedQueue.h"

#include <lcmtypes/drc/atlas_state_t.hpp>
#include <lcmtypes/drc/atlas_raw_imu_batch_t.hpp>
#include <lcmtypes/drc/robot_state_t.hpp>
#include <lcmtypes/drc/nav_state_t.hpp>
#include <lcmtypes/drc/ins_update_packet_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>

namespace StateEstimate
{

  typedef SynchronizedQueue<drc::robot_state_t> RobotStateQueue;
  typedef SynchronizedQueue<drc::atlas_state_t> AtlasStateQueue;
  typedef SynchronizedQueue<drc::atlas_raw_imu_t> IMUQueue;
  typedef SynchronizedQueue<drc::atlas_raw_imu_batch_t> IMUBatchQueue;
  typedef SynchronizedQueue<drc::nav_state_t> NavQueue;
  typedef SynchronizedQueue<drc::ins_update_packet_t> INSUpdateQueue;

  typedef SynchronizedQueue<bot_core::pose_t> PoseQueue;

  class messageQueues {  
  public:
	
	messageQueues() {
		;
	}
	  
	// note this is a reference copy constructor
	messageQueues(messageQueues &param) {
	  *this = param;
	}
	
	messageQueues& operator=(const messageQueues &rhs) {
	  //this->atlasStateQueue = 
	  // TODO -- complete this operator
	  
	  return *this;
	}
	  
    AtlasStateQueue atlasStateQueue;
    IMUQueue imuQueue;
    PoseQueue bdiPoseQueue;
    PoseQueue viconQueue;
    PoseQueue matlabTruthQueue;
  };
  
  
} // end namespace

#endif
