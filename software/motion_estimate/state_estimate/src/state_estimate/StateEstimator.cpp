#include "StateEstimator.h"

//-----------------------------------------------------------------------------
StateEstimate::StateEstimator::StateEstimator(
	const command_switches* _switches,
    boost::shared_ptr<lcm::LCM> lcmHandle,
    CommandLineConfig& cl_cfg,
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
  LegOdoWrapper(lcmHandle, lcmHandle, cl_cfg),
  inert_odo(0.001)
{

  _mSwitches = _switches;

  ERSMsgSuffix = "";
  if (_switches->ExperimentalMsgs) {
	  ERSMsgSuffix = "_EXP";
  }

  // Sandbox leg odo
  setupLegOdo();


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


  pelvis_vel_diff.setSize(3);
  d_pelvis_vel_diff.setSize(3);

  lcm = lcm_create(NULL); // Currently this pointer is not being deleted -- must be done before use
  lcmgl_ = bot_lcmgl_init(lcm, "VelArrows");

  //  lcmgl_lego = bot_lcmgl_init(lcm, "VelLegOdo");
  //  lcmgl_inerto = bot_lcmgl_init(lcm, "VelInerOdo");
  //  lcmgl_measVec = bot_lcmgl_init(lcm, "VelMeasurement");
  //  lcmgl_dV_l = bot_lcmgl_init(lcm, "VelMeasurement");

  pelvisVel_world.setZero();
  filteredPelvisVel_world.setZero();
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
	//std::cout << "StateEstimator::run -- mIMUQueue.size() " << nIMU << std::endl;
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

	//std::cout << std::endl << std::endl;
  }
}


void StateEstimate::StateEstimator::IMUServiceRoutine(const drc::atlas_raw_imu_t &imu, bool publishERSflag, boost::shared_ptr<lcm::LCM> lcm) {

  //drawInertVelArrow();

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

  //std::cout << "StateEstimator::run -- Processing new mINSUpdatePacket message, utime " << INSUpdate.utime << std::endl;

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

  // This will publish "POSE_BODY_ALT message"
  PropagateLegOdometry(bdiPose, atlasState);
  //std::cout << "StateEstimator::AtlasStateServiceRoutine" << std::endl;

  return;
  // Skipping the old stuff -- clearly needs clearing up.

  // compute the joint velocities with num_joints Kalman Filters in parallel
  // TODO -- make this dependent on local state and not the message number
  //mJointFilters.updateStates(atlasState.utime, atlasState.joint_position, atlasState.joint_velocity);
  insertAtlasState_ERS(atlasState, mERSMsg, robot);




  // TODO -- we are using the BDI orientation -- should change to either pure leg kin, combination of yaw, or V/P fused orientation
  Eigen::Quaterniond BDi_quat;
  BDi_quat.w() = bdiPose.orientation[0];
  BDi_quat.x() = bdiPose.orientation[1];
  BDi_quat.y() = bdiPose.orientation[2];
  BDi_quat.z() = bdiPose.orientation[3];
  _leg_odo->setOrientationTransform(BDi_quat, Eigen::Vector3d::Zero());
  doLegOdometry(fk_data, atlasState, bdiPose, *_leg_odo, firstpass, robot);

  // TODO -- remove this, only a temporary display object
  Eigen::Isometry3d LegOdoPelvis;
  LegOdoPelvis.setIdentity();
  LegOdoPelvis = _leg_odo->getPelvisState();
  //std::cout << "StateEstimator::AtlasStateServiceRoutine -- leg odo translation estimate " << LegOdoPelvis.translation().transpose() << std::endl;
  _leg_odo->calculateUpdateVelocityStates(atlasState.utime, LegOdoPelvis);
  std::cout << "StateEstimator::AtlasStateServiceRoutine -- leg odo pelvis velocities " << _leg_odo->getPelvisVelocityStates().transpose() << std::endl;

  bot_core::pose_t LegOdoPosMsg;

  LegOdoPosMsg.utime = atlasState.utime;
  LegOdoPosMsg.pos[0] = LegOdoPelvis.translation()(0);
  LegOdoPosMsg.pos[1] = LegOdoPelvis.translation()(1);
  LegOdoPosMsg.pos[2] = LegOdoPelvis.translation()(2);

  Eigen::Quaterniond PelvisQ;
  PelvisQ = C2q(LegOdoPelvis.linear());
  LegOdoPosMsg.orientation[0] = PelvisQ.w();
  LegOdoPosMsg.orientation[1] = PelvisQ.x();
  LegOdoPosMsg.orientation[2] = PelvisQ.y();
  LegOdoPosMsg.orientation[3] = PelvisQ.z();

  mLCM->publish("POSE_BODY_ALT", &LegOdoPosMsg);

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

InertialOdometry::DynamicState* StateEstimate::StateEstimator::getInerOdoPtr() {
  return &InerOdoEst;
}

Eigen::Vector3d* StateEstimate::StateEstimator::getFilteredLegOdoVel() {
  return &filteredPelvisVel_world;
}


void StateEstimate::StateEstimator::PropagateLegOdometry(const bot_core::pose_t &bdiPose, const drc::atlas_state_t &atlasState) {
  Eigen::Isometry3d world_to_body_bdi;
  world_to_body_bdi.setIdentity();
  //world_to_body_bdi.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(bdiPose.orientation[0], bdiPose.orientation[1], bdiPose.orientation[2], bdiPose.orientation[3]);
  world_to_body_bdi.rotate(quat);

  leg_odo_->setPoseBDI( world_to_body_bdi );
  leg_odo_->setFootForces(atlasState.force_torque.l_foot_force_z, atlasState.force_torque.r_foot_force_z);
  leg_odo_->updateOdometry(joint_utils_.atlas_joint_names, atlasState.joint_position, atlasState.utime);

  Eigen::Isometry3d world_to_body = leg_odo_->getRunningEstimate();


  pelvisVel_world = pelvis_vel_diff.diff(atlasState.utime, world_to_body.translation());
  double vel[3];
  vel[0] = lpfilter[0].processSample(pelvisVel_world(0));
  vel[1] = lpfilter[1].processSample(pelvisVel_world(1));
  vel[2] = lpfilter[2].processSample(pelvisVel_world(2));
  filteredPelvisVel_world << vel[0], vel[1], vel[2];

  bot_core::pose_t pose_msg = getPoseAsBotPose(world_to_body, atlasState.utime);
  mLCM->publish("POSE_BODY_ALT", &pose_msg );

  drawLegOdoVelArrow(world_to_body_bdi.linear());
}


void StateEstimate::StateEstimator::drawLegOdoVelArrow(const Eigen::Matrix3d &wRb_bdi) {

  Eigen::Matrix3d sRb;
  sRb << -0.707107, 0.707107, 0, 0.707107, 0.707107, 0, 0, 0, -1;
  //  sRb.setIdentity();
  //std::cout << "StateEstimator::drawVelArrows -- sRb " << std::endl << sRb << std::endl;

  double magn;
  magn = 5*filteredPelvisVel_world.norm();

  Eigen::Vector3d cp, ref;
  Eigen::Vector3d rotVel;
  double angle;

  ref << 1., 0., 0.;

  cp = ref.cross(filteredPelvisVel_world);
  cp.normalize();

  angle = acos(ref.dot(filteredPelvisVel_world)/filteredPelvisVel_world.norm());

  // euler
  //double rpy[] = {0., 3.141/4., 0.};
  //double angle;
  //double axis[3];
  //bot_roll_pitch_yaw_to_angle_axis (rpy, &angle, axis);

  //  bot_quat_to_angle_axis (const double q[4], double *theta, double axis[3]);
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_translated(lcmgl_, leg_odo_->getRunningEstimate().translation()(0), leg_odo_->getRunningEstimate().translation()(1), leg_odo_->getRunningEstimate().translation()(2));  // example offset
  bot_lcmgl_color3f(lcmgl_, 0, 1, 0); // Green
  //bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, axis[0], axis[1], axis[2]);
  bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, cp[0], cp[1], cp[2]);

  bot_lcmgl_draw_arrow_3d (lcmgl_, magn*1, magn*0.1, magn*0.2, magn*0.05);
  bot_lcmgl_pop_matrix(lcmgl_);
//  bot_lcmgl_switch_buffer(lcmgl_);

//  bot2-lcmgl/src/bot_lcmgl_client/lcmgl.h
//  void bot_lcmgl_draw_arrow_3d (bot_lcmgl_t * lcmgl, double length, double head_width, double head_length, double body_width);

  Eigen::Vector3d inerV_l;

  inerV_l = InerOdoEst.V;
  //inerV_l = rotVel;
  magn = 5*inerV_l.norm();
  //std::cout << "StateEstimate::StateEstimator::drawVelArrows -- inerV_l " << inerV_l.transpose() << std::endl;
  cp = ref.cross(inerV_l);
  cp.normalize();
  angle = acos(ref.dot(inerV_l)/inerV_l.norm());


  // euler
  //double rpy[] = {0., 3.141/4., 0.};
  //double angle;
  //double axis[3];
  //bot_roll_pitch_yaw_to_angle_axis (rpy, &angle, axis);p

  //  bot_quat_to_angle_axis (const double q[4], double *theta, double axis[3]);

  //bot_lcmgl_translated(lcmgl_inerto, leg_odo_->getRunningEstimate().translation()(0), leg_odo_->getRunningEstimate().translation()(1), leg_odo_->getRunningEstimate().translation()(2));  // example offset
  //bot_lcmgl_translated(lcmgl_inerto, 0, 0, 0);  // example offset

  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue
  //bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, axis[0], axis[1], axis[2]);
  bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, cp[0], cp[1], cp[2]);

  bot_lcmgl_draw_arrow_3d (lcmgl_, magn*1, magn*0.1, magn*0.2, magn*0.05);
  bot_lcmgl_pop_matrix(lcmgl_);
//  bot_lcmgl_switch_buffer(lcmgl_);


  // now we try a leg kin velocity vector in the IMu first posed reference frame
  // this will be in red



  Eigen::Vector3d LegOdoV_l;

  //LegOdoV_l = sRb.transpose() * wRb_bdi.transpose() * vec; // Rotate back to world frame for pretty pictures sake, but not for data fusion
  LegOdoV_l = sRb.transpose() * filteredPelvisVel_world;

  magn = 5*LegOdoV_l.norm();
  cp = ref.cross(LegOdoV_l);
  cp.normalize();
  angle = acos(ref.dot(LegOdoV_l)/LegOdoV_l.norm());

  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); // Red
  //bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, axis[0], axis[1], axis[2]);
  bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, cp[0], cp[1], cp[2]);

  bot_lcmgl_draw_arrow_3d (lcmgl_, magn*1, magn*0.1, magn*0.2, magn*0.05);
  bot_lcmgl_pop_matrix(lcmgl_);
//  bot_lcmgl_switch_buffer(lcmgl_);


  // A fourth arrow for residual velocity -- in purple

  Eigen::Vector3d dV_l;

  dV_l = LegOdoV_l - inerV_l;
  magn = 5*dV_l.norm();
  cp = ref.cross(dV_l);
  cp.normalize();
  angle = acos(ref.dot(dV_l)/dV_l.norm());

  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_translated(lcmgl_, 0, 0, 0.5);  // example offset
  bot_lcmgl_color3f(lcmgl_, 1, 0, 1); // Purple
  //bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, axis[0], axis[1], axis[2]);
  bot_lcmgl_rotated(lcmgl_, 180/3.141592*angle, cp[0], cp[1], cp[2]);

  bot_lcmgl_draw_arrow_3d (lcmgl_, magn*1, magn*0.1, magn*0.2, magn*0.05);
  bot_lcmgl_pop_matrix(lcmgl_);
  bot_lcmgl_switch_buffer(lcmgl_);

}



void StateEstimate::StateEstimator::drawInertVelArrow() {
  // inertial odometry
  double magn, angle;

  Eigen::Vector3d cp, ref;
  Eigen::Matrix3d sRb;
  sRb << -0.707107, 0.707107, 0, 0.707107, 0.707107, 0, 0, 0, -1;
  sRb.setIdentity();



}



