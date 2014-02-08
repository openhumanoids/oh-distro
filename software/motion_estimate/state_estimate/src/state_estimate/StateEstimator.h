#ifndef __StateEstimator_h
#define __StateEstimator_h

#include <vector>
#include <iostream>
#include <string>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include "ThreadLoop.h"
#include "QueueTypes.h"

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc_lcmtypes.hpp"
#include <model-client/model-client.hpp>

#include <inertial-odometry/Odometry.hpp>
#include <leg-odometry/TwoLegOdometry.h>
#include <leg-odometry/sharedUtilities.hpp>

#include "StateEstimatorUtilities.h"
#include "JointFilters.h"

#include "SharedTypes.h"

#include <drc_utils/joint_utils.hpp>
//#include "atlas/AtlasJointNames.h"

#include <estimate/LegOdoWrapper.hpp>
#include <leg-odometry/Filter.hpp>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>


namespace StateEstimate
{

class StateEstimator : public ThreadLoop, public LegOdoWrapper
{
public:


  StateEstimator(
    const StateEstimate::command_switches* _switches,
    boost::shared_ptr<lcm::LCM> lcmHandle,
    CommandLineConfig& cl_cfg_,
    AtlasStateQueue& atlasStateQueue,
    IMUQueue& imuQueue,
    PoseQueue& bdiPoseQueue,
    PoseQueue& viconQueue,
    NavQueue& matlabTruthQueue,
    INSUpdateQueue& INSUpdateQueue);
  
  StateEstimator(
      boost::shared_ptr<lcm::LCM> lcmHandle,
      messageQueues& msgQueue );

  ~StateEstimator();

  InertialOdometry::Odometry* getInertialOdometry();
  drc::robot_state_t* getERSMsg();
  drc::ins_update_request_t* getDataFusionReqMsg();
  InertialOdometry::DynamicState* getInerOdoPtr();
  Eigen::Vector3d* getFilteredLegOdoVel();

protected:

  void run();

  boost::shared_ptr<lcm::LCM> mLCM;

  AtlasStateQueue& mAtlasStateQueue;
  IMUQueue& mIMUQueue;
  PoseQueue& mBDIPoseQueue;
  PoseQueue& mViconQueue;
  NavQueue& mMatlabTruthQueue;
  INSUpdateQueue& mINSUpdateQueue;
  
  // This class is not up and running yet
  messageQueues mMSGQueues;
  
  JointFilters mJointFilters;
  //  vector<float> mJointVelocities;
  
  RobotModel* robot;

  
private:
  const StateEstimate::command_switches* _mSwitches;
  std::string ERSMsgSuffix;

  drc::robot_state_t mERSMsg;
  //drc::robot_state_t testing;
  drc::ins_update_request_t mDFRequestMsg;
	
  // We maintain own InerOdoEst state, to avoid potential future multi-threaded issues
  InertialOdometry::DynamicState InerOdoEst;
  InertialOdometry::Odometry inert_odo;
  unsigned long long previous_imu_utime;
  unsigned long long prevImuPacketCount;

  BotParam* _botparam;
  BotFrames* _botframes;
  

  // LegOdometry Object
  Eigen::Vector3d pelvisVel_world, filteredPelvisVel_world;

  // Both these leg_odo objects are to be depreciated
  TwoLegs::TwoLegOdometry *_leg_odo; // VRC version
  //  leg_odometry* leg_odo_sandbox;
  TwoLegs::FK_Data fk_data;
  

  int firstpass;
  double Ts_imu; // Auto-detect the sample rate of the IMU
  int receivedIMUPackets;

  JointUtils joint_utils_;

  NumericalDiff pelvis_vel_diff;
  DistributedDiff d_pelvis_vel_diff;

  LowPassFilter lpfilter[3];

  lcm_t* lcm;
  bot_lcmgl_t* lcmgl_;
  bot_lcmgl_t* lcmgl_lego;
  bot_lcmgl_t* lcmgl_inerto;
  bot_lcmgl_t* lcmgl_measVec;
  bot_lcmgl_t* lcmgl_dV_l;


  // ======== SERVICE ROUTINES ===========
  void IMUServiceRoutine(const drc::atlas_raw_imu_t &imu, bool publishERSflag, boost::shared_ptr<lcm::LCM> lcm);
  void INSUpdateServiceRoutine(const drc::ins_update_packet_t &INSUpdate);
  void AtlasStateServiceRoutine(const drc::atlas_state_t &atlasState, const bot_core::pose_t &bdiPose);
  void PropagateLegOdometry(const bot_core::pose_t &bdiPose, const drc::atlas_state_t &atlasState);

  //========= Some Utilities =============
  void drawLegOdoVelArrow(const Eigen::Matrix3d &wRb_bdi);
  void drawInertVelArrow();
};

} // end namespace

#endif // __StateEstimator_h
