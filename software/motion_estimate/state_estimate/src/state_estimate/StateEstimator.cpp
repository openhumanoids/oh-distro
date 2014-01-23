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
  mINSUpdateQueue(INSUpdateQueue)
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
  
  // Go get the joint names for FK
    robot = new RobotModel;
	lcm::Subscription* robot_model_subcription_;
	robot_model_subcription_ = robot->lcm.subscribeFunction("ROBOT_MODEL", StateEstimate::onMessage, robot);
	while(robot->lcm.handle()==-1);// wait for one message, wait until you get a success.
	robot->lcm.unsubscribe(robot_model_subcription_);


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

		  std::cout << "StateEstimator::run -- dequeued IMU utime " << imu.utime << std::endl;

		  // do something with new imu...
		  // The inertial odometry object is currently passed down to the handler function, where the INS is propagated and the 12 states are inserted inthe ERS message

		  //std::cout << "StateEstimator::run -- imu.packet_count " << imu.packet_count << std::endl;
		  // Auto-detect the sample rate of the IMU -- mainly for testing from different IMUs, should be 1kHz for KVH on Atlas and probably 100Hz for Microstrain
		  unsigned long long deltaPacket;
		  if (imu.packet_count > prevImuPacketCount) {
			  deltaPacket = imu.packet_count - prevImuPacketCount;
			  if (deltaPacket > 1) {
				  std::cout << "StateEstimator::run -- " << deltaPacket-1 << " missing IMU packets!" << std::endl;
			  }
			  else
			  {
				  if (receivedIMUPackets < 10) {
					  receivedIMUPackets++;
					  Ts_imu = (imu.utime - previous_imu_utime)*1.E-6/deltaPacket;
					  previous_imu_utime = imu.utime;
					  std::cout << "StateEstimator::run -- deltaPacket computed as: " << deltaPacket << ", Ts_imu set to " << Ts_imu << std::endl;
				  }
			  }
		  } else {
			  if (prevImuPacketCount != 0) {
				  std::cerr << "StateEstimator::run -- non-monotonic IMU packet count!!! assuming " << Ts_imu << " s spacing between packets." << std::endl;
			  }
		  }
		  prevImuPacketCount = imu.packet_count;

		  handle_inertial_data_temp_name(Ts_imu, imu, bdiPose, IMU_to_body, inert_odo, mERSMsg, mDFRequestMsg, _leg_odo);
		  std::cout << "StateEstimator::run -- new IMU message, utime: " << imu.utime << std::endl;

		  // TODO -- We should wait on IMU message, not AtlasState
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

			  // Insert the required inertial data in the data fusion update request message
			  stampInertialPoseUpdateRequestMsg(inert_odo, mDFRequestMsg);

//			  if (_mSwitches->MATLAB_MotionSimulator) {
////				  stampMatlabReferencePoseUpdateRequest(matlabPose, mDFRequestMsg);
//			  } else {
			  	  stampEKFReferenceMeasurementUpdateRequest(Eigen::Vector3d::Zero(), drc::ins_update_request_t::VELOCITY_LOCAL, mDFRequestMsg);
//			  }

			  // This message will contain reference measurement information from various sources -- for now it is LegOdo, Fovis, MatlabtrajectorMotionSimulation
			  mLCM->publish("SE_MATLAB_DATAFUSION_REQ", &mDFRequestMsg);
		  }
	  }



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
      insertAtlasState_ERS(atlasState, mERSMsg, robot);
      // std::cout << "Handled Atlas state" << std::endl;
      
      // here we compute the leg odometry position solution
      // TODO -- we are using the BDI orientation estimate to 
      
      doLegOdometry(fk_data, atlasState, bdiPose, *_leg_odo, firstpass, robot);

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

	  // do something with new MatlabTruthPose...
	  //std::cout << "StateEstimator::run -- Processing new mINSUpdatePacket message, utime " << INSUpdate.utime << std::endl;
	  //std::cout << "StateEstimator::run -- Processing new mINSUpdatePacket dbg " << INSUpdate.dbiasGyro_b.x << ", " << INSUpdate.dbiasGyro_b.y << ", " << INSUpdate.dbiasGyro_b.z << std::endl;

	  InertialOdometry::INSUpdatePacket insUpdatePacket;
	  insUpdatePacket.utime = INSUpdate.utime;
	  insUpdatePacket.dbiasGyro_b(0) = INSUpdate.dbiasGyro_b.x;
	  insUpdatePacket.dbiasGyro_b(1) = INSUpdate.dbiasGyro_b.y;
	  insUpdatePacket.dbiasGyro_b(2) = INSUpdate.dbiasGyro_b.z;

	  insUpdatePacket.dbiasAcc_b << INSUpdate.dbiasAcc_b.x, INSUpdate.dbiasAcc_b.y, INSUpdate.dbiasAcc_b.z;

	  // Temporary addition for debugging
	  //insUpdatePacket.dbiasGyro_b.setZero();
	  //insUpdatePacket.dbiasAcc_b.setZero();

	  insUpdatePacket.dE_l(0) = INSUpdate.dE_l.x;
	  insUpdatePacket.dE_l(1) = INSUpdate.dE_l.y;
	  insUpdatePacket.dE_l(2) = INSUpdate.dE_l.z;

	  insUpdatePacket.dVel_l << INSUpdate.dVel_l.x, INSUpdate.dVel_l.y, INSUpdate.dVel_l.z;
	  insUpdatePacket.dPos_l << INSUpdate.dPos_l.x, INSUpdate.dPos_l.y, INSUpdate.dPos_l.z;

	  // And here we finally roll in the updates to the InertialOdometry INS prediction
	  inert_odo.incorporateERRUpdate(insUpdatePacket);
	}

    // add artificial delay
    //boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  }
}
