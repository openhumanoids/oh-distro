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
//#include <leg-odometry/TwoLegOdometry.h>
#include <leg-odometry/sharedUtilities.hpp>
#include <leg-odometry/SignalTap.hpp>

#include "StateEstimatorUtilities.h"
#include "JointFilters.h"

#include "SharedTypes.h"

#include <drc_utils/joint_utils.hpp>
//#include "atlas/AtlasJointNames.h"

#include <basiclegodo/LegOdoWrapper.hpp>
#include <leg-odometry/Filter.hpp>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>


namespace StateEstimate
{

#define MIN_STANDING_CLASSIF_FORCE  200.
#define MIN_WALKING_FORCE            50.
#define MAX_STANDING_SPEED            0.02
#define STANDING_TIMEOUT		 100000
#define WALKING_TIMEOUT		     300000


class StateEstimator : public ThreadLoop, public LegOdoWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  int* getLegStateClassificationPtr();
  Eigen::Isometry3d* getVelArrowDrawTransform();
  Eigen::Isometry3d* getAlignTransform();

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
  Eigen::Vector3d pelvisVel_world, filteredPelvisVel_world, imuVel_world;

  // Both these leg_odo objects are to be depreciated
  //TwoLegs::TwoLegOdometry *_leg_odo; // VRC version
  //  leg_odometry* leg_odo_sandbox;
  //TwoLegs::FK_Data fk_data;
  

  int firstpass;
  double Ts_imu; // Auto-detect the sample rate of the IMU
  int receivedIMUPackets;

  JointUtils joint_utils_;

  NumericalDiff pelvis_vel_diff; // Will probably be depreciated, since we are interested in the aided IMU velocity
  NumericalDiff imu_vel_diff;
  DistributedDiff d_pelvis_vel_diff;

  LowPassFilter lpfilter[3];

  lcm_t* lcm;
  bot_lcmgl_t* lcmgl_;
  Eigen::Isometry3d velArrowTransform;
  Eigen::Isometry3d align;


  // ======== SERVICE ROUTINES ===========
  void IMUServiceRoutine(const drc::atlas_raw_imu_t &imu, bool publishERSflag, boost::shared_ptr<lcm::LCM> lcm);
  void INSUpdateServiceRoutine(const drc::ins_update_packet_t &INSUpdate);
  void AtlasStateServiceRoutine(const drc::atlas_state_t &atlasState, const bot_core::pose_t &bdiPose);
  void PropagateLegOdometry(const bot_core::pose_t &bdiPose, const drc::atlas_state_t &atlasState);

  // ======== Classifiers ================
  int mLegStateClassification;
  int classifyDynamicLegState(const unsigned long long &uts, const double forces[2], const double &speed);

  // Utility function used to determine the state of the robot
  ExpireTimer standingTimer;
  bool standingClassifier(const unsigned long long &uts, const double forces[2], const double &speed);
  ExpireTimer velUpdateTimer;
  bool velocityUpdateClassifier(const unsigned long long &uts, const double forces[2], const double &speed);


  // ======== Some Utilities =============
  void drawLegOdoVelArrow(const Eigen::Matrix3d &wRb_bdi);


};

} // end namespace

#endif // __StateEstimator_h
