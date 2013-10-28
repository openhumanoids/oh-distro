#include "StateEstimator.h"

//-----------------------------------------------------------------------------
StateEstimate::StateEstimator::StateEstimator(
	const command_switches* _switches,
    boost::shared_ptr<lcm::LCM> lcmHandle,
    AtlasStateQueue& atlasStateQueue,
    IMUQueue& imuQueue,
    PoseQueue& bdiPoseQueue,
    PoseQueue& viconPoseQueue ) :

  mLCM(lcmHandle),
  mAtlasStateQueue(atlasStateQueue),
  mIMUQueue(imuQueue),
  mBDIPoseQueue(bdiPoseQueue),
  mViconQueue(viconPoseQueue)
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
  std::cout << "StateEstimator::StateEstimator -- IMU_to_body: " << IMU_to_body.linear() << std::endl << IMU_to_body.translation() << std::endl;
  
  std::cout << "StateEstimator::StateEstimator -- Creating new TwoLegOdometry object." << std::endl;
  // using this constructor as a bit of legacy -- but in reality we should probably inprove on this situation
  _leg_odo = new TwoLegs::TwoLegOdometry(false, false, 1400.f);
  
  unsigned long fusion_period;
  fusion_period = 20000-500;
  fusion_rate.setDesiredPeriod_us(0,fusion_period);
  fusion_rate.setSize(1);
  fusion_rate_dummy.resize(1);
  fusion_rate_dummy << 0;
  std::cout << "StateEstimator::StateEstimator -- Setting data fusion period trigger is set to " << fusion_period << std::endl;
 
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
}

//-----------------------------------------------------------------------------
void StateEstimate::StateEstimator::run()
{
  drc::atlas_state_t atlasState;
  drc::atlas_raw_imu_t imu;
  bot_core::pose_t bdiPose;
  bot_core::pose_t viconPose;

  while (!this->ShouldStop)
  {

    // wait for at least one new atlas_state message
	  // TODO -- Pat please make this pass on any event
    //this->mAtlasStateQueue.waitWhileEmpty();
	  this->mIMUQueue.waitWhileEmpty();


    int nAtlasStates = mAtlasStateQueue.size();
    for (int i = 0; i < nAtlasStates; ++i)
    {
      this->mAtlasStateQueue.dequeue(atlasState);

      // do something with new atlas state...
      
      // Here we compute the joint velocities with num_joints Kalman Filters in parallel
      // TODO -- dehann, make this dependent on local state and not the message number
      //float joint_velocities[atlasState.num_joints];
      // Joint velocity states in the atlasState message are overwriten by this process
      mJointFilters.updateStates(atlasState.utime, atlasState.joint_position, atlasState.joint_velocity);
      insertAtlasState_ERS(atlasState, mERSMsg);
      // std::cout << "Handled Atlas state" << std::endl;
      
      // here we compute the leg odometry position solution
      // TODO -- we are using the BDI orientation estimate to 
      
      doLegOdometry(fk_data, atlasState, bdiPose, *_leg_odo, firstpass);

      // TODO -- remove this, only a temporary display object
      // Tihs is where leg odometry thinks the pelvis is at
      Eigen::Isometry3d LegOdoPelvis;
      LegOdoPelvis.setIdentity();
      LegOdoPelvis = _leg_odo->getPelvisState();
      std::cout << "StateEstimator::run -- leg odo translation estimate " << LegOdoPelvis.translation().transpose() << std::endl;
      
      // This is the counter we use to initialize the pose of the robot at start of the state-estimator process
      if (firstpass>0)
        firstpass--;
    }

    // This is the special case which will also publish the message
    int nIMU = mIMUQueue.size();
    std::cout << "StateEstimator::run -- mIMUQueue.size() " << nIMU << std::endl;
    // printf("have %d new imu\n", nIMU);
    for (int i = 0; i < nIMU; ++i)
    {
      this->mIMUQueue.dequeue(imu);

      std::cout << "StateEstimator::run -- dequeued IMU utime " << imu.utime << std::endl;

      // do something with new imu...
      // The inertial odometry object is currently passed down to the handler function, where the INS is propagated and the 12 states are inserted inthe ERS message
      double dt;
      dt = (imu.utime - previous_imu_utime)*1.E-6;
      previous_imu_utime = imu.utime;
      handle_inertial_data_temp_name(dt, imu, bdiPose, IMU_to_body, inert_odo, mERSMsg, mDFRequestMsg);
      
      std::cout << "StateEstimator::run -- new IMU message, utime: " << imu.utime << std::endl;

      // TODO -- Pat, dehann: we need to do this publishing in a better manner. We should wait on IMU message, not AtlasState
      // For now we are going to publish on the last element of this queue
      //      std::cout << "StateEstimator::run -- nIMU = " << nIMU << std::endl;
      if (i==(nIMU-1)) { 
	      //publish ERS message
    	  std::cout << std::endl << std::endl << "Going to publish ERS" << std::endl;
    	  mERSMsg.utime = imu.utime;
	      mLCM->publish("EST_ROBOT_STATE" + ERSMsgSuffix, &mERSMsg); // There is some silly problem here

	      mDFRequestMsg.updateType = mDFRequestMsg.INS_POSE_ONLY;
	      mLCM->publish("SE_INS_POSE_STATE", &mDFRequestMsg);
      }

      if (fusion_rate.genericRateChange(imu.utime,fusion_rate_dummy,fusion_rate_dummy)) {
	    std::cout << "StateEstimator::run -- data fusion message is being sent with time " << imu.utime << std::endl;

	    // populate the INS state information and the measurement aiding information
	    packDFUpdateRequestMsg(inert_odo, *_leg_odo, mDFRequestMsg);
	    mLCM->publish("SE_MATLAB_DATAFUSION_REQ", &mDFRequestMsg);
	  }

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

    // add artificial delay
    //boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  }
}
