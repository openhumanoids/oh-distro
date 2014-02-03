#include "StateEstimator.h"

//-----------------------------------------------------------------------------
StateEstimate::StateEstimator::StateEstimator(
	const command_switches* _switches,
    boost::shared_ptr<lcm::LCM> lcmHandle,
    AtlasStateQueue& atlasStateQueue,
    IMUQueue& imuQueue,
    PoseQueue& bdiPoseQueue,
    PoseQueue& viconPoseQueue,
    NavQueue& viconMatlabtruthQueue,
    INSUpdateQueue& INSUpdateQueue) :

  mLCM(lcmHandle),
  mAtlasStateQueue(atlasStateQueue),
  mIMUQueue(imuQueue),
  mBDIPoseQueue(bdiPoseQueue),
  mViconQueue(viconPoseQueue),
  mMatlabTruthQueue(viconMatlabtruthQueue),
  mINSUpdateQueue(INSUpdateQueue),
  inert_odo(0.01)
{

  _mSwitches = _switches;

  ERSMsgSuffix = "";
  if (_switches->ExperimentalMsgs) {
	  ERSMsgSuffix = "_EXP";
  }

  // TODO -- dehann, this should be initialized to the number of joints in the system, but just hacking to get it going for now
  int num_joints = 28;
  
  mJointFilters.setSize(num_joints);
  //  mJointVelocities.resize(num_joints);
  std::cout << "StateEstimator::StateEstimator -- hardcoded the number of joint Kalman Filters to " << num_joints << std::endl;
  std::cerr << "StateEstimator::StateEstimator -- hardcoded the number of joint Kalman Filters to " << num_joints << std::endl;
  
  // get the imu to body transform
  _botparam = bot_param_new_from_server(mLCM->getUnderlyingLCM(), 0);
  _botframes= bot_frames_get_global(mLCM->getUnderlyingLCM(), _botparam);

  Eigen::Isometry3d IMU_to_body;
  // Define the transform between the pelvis and IMU -- considering that we purposefull skip this transform during development
  IMU_to_body.setIdentity();
  if (_mSwitches->MATLAB_MotionSimulator == false) {
	// This for running on the real robot
	int status;
	double matx[16];
	status = bot_frames_get_trans_mat_4x4_with_utime( _botframes, "body",  "imu", 0 /*utime*/, matx);
	for (int i = 0; i < 4; ++i) {
	  for (int j = 0; j < 4; ++j) {
		IMU_to_body(i,j) = matx[i*4+j];
	  }
	}
  }

  // TEMPORARY
  //IMU_to_body.setIdentity();
  inert_odo.setIMU2Body(Eigen::Isometry3d::Identity());
  
  // Go get the joint names for FK
    robot = new RobotModel;
	lcm::Subscription* robot_model_subcription_;
	robot_model_subcription_ = robot->lcm.subscribeFunction("ROBOT_MODEL", StateEstimate::onMessage, robot);
	while(robot->lcm.handle()==-1);// wait for one message, wait until you get a success.
	robot->lcm.unsubscribe(robot_model_subcription_);


  std::cout << "StateEstimator::StateEstimator -- Creating new TwoLegOdometry object." << std::endl;
  // using this constructor as a bit of legacy -- but in reality we should probably inprove on this situation
  _leg_odo = new TwoLegs::TwoLegOdometry(false, false, 1400.f);
  

  // This is for forward kinematics -- maybe not the best way to do this, but we are a little short on time. Code evolution will fix this in the long run
  fk_data.model_ = boost::shared_ptr<ModelClient>(new ModelClient(mLCM->getUnderlyingLCM(), 0));
  // Parse KDL tree
  if (!kdl_parser::treeFromString(  fk_data.model_->getURDFString() , fk_data.tree)){
	std::cerr << "StateEstimator::StateEstimator -- ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
	return;
  }
  fk_data.fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(fk_data.tree));
  
  // This is used to initialize the states of the robot -- note we should use ONLY this variable
  firstpass = 1;
  prevImuPacketCount = 0;
  Ts_imu = 1E-3;
  receivedIMUPackets = 0;
}

// TODO -- fix this constructor
//StateEstimate::StateEstimator::StateEstimator(
//    boost::shared_ptr<lcm::LCM> lcmHandle,
//    messageQueues& msgQueue ) :
//  
//  mLCM(lcmHandle),
//  mMSGQueues(msgQueue)
//{
//
//}

//-----------------------------------------------------------------------------
StateEstimate::StateEstimator::~StateEstimator()
{
  delete _leg_odo;
  delete robot;
}

//-----------------------------------------------------------------------------
void StateEstimate::StateEstimator::run()
{
  drc::atlas_state_t atlasState;
  drc::atlas_raw_imu_t imu;
  drc::nav_state_t matlabPose;
  drc::ins_update_packet_t INSUpdate;
  bot_core::pose_t bdiPose;
  bot_core::pose_t viconPose;


  while (!this->ShouldStop)
  {

    // wait for at least one new atlas_state message
	  // TODO -- Pat please make this pass on any event
    //this->mAtlasStateQueue.waitWhileEmpty();
	this->mIMUQueue.waitWhileEmpty();


	// This is the special case which will also publish the message
	int nIMU = mIMUQueue.size();
	std::cout << "StateEstimator::run -- mIMUQueue.size() " << nIMU << std::endl;
	// printf("have %d new imu\n", nIMU);
	for (int i = 0; i < nIMU; ++i)
	{
	  this->mIMUQueue.dequeue(imu);
	  //std::cout << "StateEstimator::run -- new IMU message, utime: " << imu.utime << std::endl;
	  // Handle IMU data -- Special case, this one retransmits ERS and INSUpdateRequest messages internally
	  IMUServiceRoutine(imu, (i==(nIMU-1)), mLCM);
	}

    int nAtlasStates = mAtlasStateQueue.size();
    for (int i = 0; i < nAtlasStates; ++i)
    {
      this->mAtlasStateQueue.dequeue(atlasState);
      // Handle atlasState message
      AtlasStateServiceRoutine(atlasState, bdiPose);
    }

    const int nPoses = mBDIPoseQueue.size();
    for (int i = 0; i < nPoses; ++i)
    {
      mBDIPoseQueue.dequeue(bdiPose);
      // push bdiPose info into ERS
      convertBDIPose_ERS(&bdiPose, mERSMsg);
    }

    const int nViconPoses = mViconQueue.size();
    for (int i = 0; i < nViconPoses; ++i)
    {
      mViconQueue.dequeue(viconPose);
      // do something with new vicon pose...

    }

    const int nMatlabTruth = mMatlabTruthQueue.size();
	for (int i = 0; i < nMatlabTruth; ++i)
	{
		mMatlabTruthQueue.dequeue(matlabPose);

	  // do something with new MatlabTruthPose...
	  std::cout << "StateEstimator::run -- Processing new matlabTruthPose message" << std::endl;
	}

	const int nINSUpdates = mINSUpdateQueue.size();
	for (int i = 0; i < nINSUpdates; ++i)
	{
	  mINSUpdateQueue.dequeue(INSUpdate);
	  // Handle the INS update request
	  INSUpdateServiceRoutine(INSUpdate);

	}

	std::cout << std::endl << std::endl;
  }
}


void StateEstimate::StateEstimator::IMUServiceRoutine(const drc::atlas_raw_imu_t &imu, bool publishERSflag, boost::shared_ptr<lcm::LCM> lcm) {
  //
  //	if (receivedIMUPackets < 10) {
  //		detectIMUSampleTime(prevImuPacketCount, previous_imu_utime, receivedIMUPackets, Ts_imu, imu);
  //		std::cout << "StateEstimator::run -- auto-detecting IMU sample time at " << Ts_imu << " s" << std::endl;
  //	}
  //std::cout << "StateEstimator::run -- Ts_imu set to " << Ts_imu << " s" << std::endl; // Remove once confirmed to be working properly

  // EKF measurement update rate set to 20ms here

	//InerOdoEst = PropagateINS(Ts_imu, inert_odo, IMU_to_body, imu);

//	if (publishERSflag) {
//	  //publish ERS message on last IMU message in the current queue
//      stampInertialPoseERSMsg(InerOdoEst, mERSMsg);
//      std::cout << "StateEstimator::run -- Publish ERS" << std::endl;
//      lcm->publish("EST_ROBOT_STATE" + ERSMsgSuffix, &mERSMsg);
//      //lcm->publish("POSE_BODY", &??);
//	}

	// Request an update to the INS state -- This publish should be blocked until publishing of ERS message
//	if (fusion_rate.genericRateChange(imu.utime,fusion_rate_dummy,fusion_rate_dummy)) {
//		std::cout << "StateEstimator::run -- data fusion message is being sent with time " << imu.utime << std::endl;
//		stampInertialPoseUpdateRequestMsg(InerOdoEst, mDFRequestMsg);
//		stampEKFReferenceMeasurementUpdateRequest(Eigen::Vector3d::Zero(), drc::ins_update_request_t::VELOCITY_LOCAL, mDFRequestMsg);
//		lcm->publish("SE_MATLAB_DATAFUSION_REQ", &mDFRequestMsg);
//	}
}


void StateEstimate::StateEstimator::INSUpdateServiceRoutine(const drc::ins_update_packet_t &INSUpdate) {

  std::cout << "StateEstimator::run -- Processing new mINSUpdatePacket message, utime " << INSUpdate.utime << std::endl;

  InertialOdometry::INSUpdatePacket insUpdatePacket;
  insUpdatePacket.utime = INSUpdate.utime;
  insUpdatePacket.dbiasGyro_b << INSUpdate.dbiasGyro_b.x, INSUpdate.dbiasGyro_b.y, INSUpdate.dbiasGyro_b.z;
  insUpdatePacket.dbiasAcc_b << INSUpdate.dbiasAcc_b.x, INSUpdate.dbiasAcc_b.y, INSUpdate.dbiasAcc_b.z;
  insUpdatePacket.dE_l << INSUpdate.dE_l.x, INSUpdate.dE_l.y, INSUpdate.dE_l.z;
  insUpdatePacket.dVel_l << INSUpdate.dVel_l.x, INSUpdate.dVel_l.y, INSUpdate.dVel_l.z;
  insUpdatePacket.dPos_l << INSUpdate.dPos_l.x, INSUpdate.dPos_l.y, INSUpdate.dPos_l.z;

  // And here we finally roll in the updates to the InertialOdometry INS prediction
  inert_odo.incorporateERRUpdate(insUpdatePacket);
}

void StateEstimate::StateEstimator::AtlasStateServiceRoutine(const drc::atlas_state_t &atlasState, const bot_core::pose_t &bdiPose) {

  // compute the joint velocities with num_joints Kalman Filters in parallel
  // TODO -- make this dependent on local state and not the message number
  //mJointFilters.updateStates(atlasState.utime, atlasState.joint_position, atlasState.joint_velocity);
  insertAtlasState_ERS(atlasState, mERSMsg, robot);

  // TODO -- we are using the BDI orientation -- should change to either pure leg kin, combination of yaw, or V/P fused orientation
  doLegOdometry(fk_data, atlasState, bdiPose, *_leg_odo, firstpass, robot);

  // TODO -- remove this, only a temporary display object
  Eigen::Isometry3d LegOdoPelvis;
  LegOdoPelvis.setIdentity();
  LegOdoPelvis = _leg_odo->getPelvisState();
  std::cout << "StateEstimator::run -- leg odo translation estimate " << LegOdoPelvis.translation().transpose() << std::endl;

  // This is the counter we use to initialize the pose of the robot at start of the state-estimator process
  if (firstpass>0)
	firstpass--;
}

InertialOdometry::Odometry* StateEstimate::StateEstimator::getInertialOdometry() {
  return &inert_odo;
}

drc::robot_state_t* StateEstimate::StateEstimator::getERSMsg() {
  return &mERSMsg;
}

drc::ins_update_request_t* StateEstimate::StateEstimator::getDataFusionReqMsg() {
  return &mDFRequestMsg;
}

