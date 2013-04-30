
/*
 * Refer to "LegOdometry_LCM_Handler.h" for comments on the purpose of each function and member, comments here in the .cpp relate
 * to more development issues and specifics. The user only need worry about the definitions and descriptions given in the header file,
 * assuming a good software design was done.
 * d fourie
 * 3/24/2013
*/

#include <iostream>
#include <exception>
//#include <stdio.h>
//#include <inttypes.h>
#include <time.h>

#include "LegOdometry_LCM_Handler.hpp"
#include "QuaternionLib.h"




using namespace TwoLegs;
using namespace std;

LegOdometry_Handler::LegOdometry_Handler(boost::shared_ptr<lcm::LCM> &lcm_, command_switches* commands):
        _finish(false), lcm_(lcm_) {
	// Create the object we want to use to estimate the robot's pelvis position
	// In this case its a two legged vehicle and we use TwoLegOdometry class for this task
	
	_switches = commands;
	
	std::cout << "Switches value for listen to LCM trues is: " << _switches->lcm_read_trues << std::endl;
	
	_leg_odo = new TwoLegOdometry(_switches->log_data_files);
	
	 _botparam = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
	 _botframes= bot_frames_get_global(lcm_->getUnderlyingLCM(), _botparam);
	
	
	ratecounter = 0;
	local_to_head_vel_diff.setSize(3);
	local_to_head_acc_diff.setSize(3);
	local_to_head_rate_diff.setSize(3);
	
	if (_switches->lcm_add_ext) {
	  _channel_extension = "";
	} else {
		_channel_extension = "_VO";
	}
	
	if(!lcm_->good())
	  return;
	
	model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
	
#ifdef VERBOSE_DEGUG
	std::cout << "LegOdometry_handler is now subscribing to LCM messages: " << "TRUE_ROBOT_STATE, " << "TORSO_IMU" << std::endl; 
#endif
	
	lcm_->subscribe("TRUE_ROBOT_STATE",&LegOdometry_Handler::robot_state_handler,this);
	lcm_->subscribe("TORSO_IMU",&LegOdometry_Handler::torso_imu_handler,this);
	
	/*
	if (_switches->lcm_read_trues) {
		// now we can listen to the true values of POSE_HEAD
		lcm_->subscribe("POSE_HEAD_TRUE", &LegOdometry_Handler::pose_head_true_handler, this);
	}
	*/
	
	// Parse KDL tree
	  if (!kdl_parser::treeFromString(  model_->getURDFString() ,tree)){
	    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
	    return;
	  }
	  
	  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

	
	stillbusy = false;
	
	// This is for viewing results in the collections_viewer. check delete of new memory
	lcm_viewer = lcm_create(NULL);
	
	poseplotcounter = 0;
	collectionindex = 101;
	_obj = new ObjectCollection(1, std::string("Objects"), VS_OBJ_COLLECTION_T_POSE3D);
	_link = new LinkCollection(2, std::string("Links"));
	
	firstpass = true;
	
//#if defined( DISPLAY_FOOTSTEP_POSES ) || defined( DRAW_DEBUG_LEGTRANSFORM_POSES )
  if (_switches->draw_footsteps) {
	_viewer = new Viewer(lcm_viewer);
  }
	//#endif
  
    state_estimate_error_log.Open(_switches->log_data_files,"estimated_state_errors.csv");
	
	return;
}

LegOdometry_Handler::~LegOdometry_Handler() {
	
	  state_estimate_error_log.Close();
	
	//delete model_;
	delete _leg_odo;
	delete _obj;
	delete _link;
	
	lcm_destroy(lcm_viewer); //destroy viewer memory at executable end
	delete _viewer;
	
	// Not sure if the pointers must be deleted here, this may try and delete a pointer to a static memory location -- therefore commented
	//delete _botframes;
	//delete _botparam;
	
	cout << "Everything Destroyed in LegOdometry_Handler::~LegOdometry_Handler()" << endl;
	return;
}

void LegOdometry_Handler::setupLCM() {
	
//	_lcm = lcm_create(NULL);
	// TODO
	// robot_pose_channel = "TRUE_ROBOT_STATE";
	// drc_robot_state_t_subscribe(_lcm, robot_pose_channel, TwoLegOdometry::on_robot_state_aux, this);
	
	
	return;
}
			

void LegOdometry_Handler::run(bool testingmode) {
	
	// TODO
	cout << "LegOdometry_Handler::run(bool) is NOT finished yet." << endl;
	
	if (testingmode)
	{
		cout << "LegOdometry_Handler::run(bool) in tesing mode." << endl;
		
		for (int i = 0 ; i<10 ; i++)
		{
			_leg_odo->CalculateBodyStates_Testing(i);
			
		}
		
		
	}
	else
	{
		cout << "Attempting to start lcm_handle loop..." << endl;
		
		try
		{
			// This is the highest referrence point for the 
			//This is in main now...
			//while(0 == lcm_->handle());
		    
		}
		catch (exception& e)
		{
			cout << "LegOdometry_Handler::run() - Oops something went wrong when we tried to listen and respond to a new lcm message:" << endl;
			cout << e.what() << endl;
			
		}
	}
	
	return;
}

// To be moved to a better abstraction location
void LegOdometry_Handler::DetermineLegContactStates(long utime, float left_z, float right_z) {
	// The idea here is to determine the contact state of each foot independently
	// to enable better initialization logic when the robot is stating up, or stading up after falling down
	_leg_odo->updateSingleFootContactStates(utime, left_z, right_z);
}

void LegOdometry_Handler::ParseFootForces(const drc::robot_state_t* msg, double &left_force, double &right_force) {
	left_force = msg->contacts.contact_force[0].z;
	right_force = msg->contacts.contact_force[1].z;
}


void LegOdometry_Handler::robot_state_handler(	const lcm::ReceiveBuffer* rbuf, 
												const std::string& channel, 
												const  drc::robot_state_t* msg) {
	ratecounter++;
	if (ratecounter < 1)
		return;
	//std::cout << "Timestamp: " << msg->utime << std::endl;
	ratecounter = 0;
	
	//timespec before, quater,mid, threequat, after;
	//clock_gettime(CLOCK_REALTIME, &before);
	
	bool legchangeflag;

	Eigen::Isometry3d left;
	Eigen::Isometry3d right;
	
	getTransforms(msg,left,right);
	
	// TODO -- get the name based left and right foot force data here
	double left_force, right_force;
	ParseFootForces(msg, left_force, right_force);
	left_force = msg->contacts.contact_force[0].z;
	right_force = msg->contacts.contact_force[1].z;

	if (_switches->publish_footcontact_states) {
	  DetermineLegContactStates((long)msg->utime,left_force,right_force); // should we have a separate foot contact state classifier, which is not embedded in the leg odometry estimation process	
	  PublishFootContactEst(msg->utime);
	}
	
	if (_switches->do_estimation){
		// This will have to change to something which checks if the feet are in contact. a transition from a free to contact state should trigger the initial state reset
		if (firstpass)
		{
			firstpass = false;
			// We need to reset the initial condition of the odometry estimate here..
			// TODO -- can we access the true first state, or do we just follow this for initial testing. im sure we are going to have to remove this before the VRC
			Eigen::Quaterniond dummy_var;
	        dummy_var.w() = msg->origin_position.rotation.w;
	        dummy_var.x() = msg->origin_position.rotation.x;
	        dummy_var.y() = msg->origin_position.rotation.y;
	        dummy_var.z() = msg->origin_position.rotation.z;
	        
			Eigen::Isometry3d first_pose_init(dummy_var);
			first_pose_init.translation().x() = msg->origin_position.translation.x;
			first_pose_init.translation().y() = msg->origin_position.translation.y;
			first_pose_init.translation().z() = msg->origin_position.translation.z;
			
			_leg_odo->ResetWithLeftFootStates(left,right,first_pose_init);
			//std::cout << "Footsteps initialized, pelvis at: " << _leg_odo->getPelvisFromStep().translation().transpose() <<"\n";
			
		}
		_leg_odo->setLegTransforms(left, right);
		
		legchangeflag = _leg_odo->FootLogic(msg->utime, left_force, right_force);
	
		// Timing profile. This is the midway point
		//clock_gettime(CLOCK_REALTIME, &mid);
		
		// Think we should add the velocity estimation process here
		_leg_odo->calculateUpdateVelocityStates(msg->utime);
		
		
#ifdef DRAW_DEBUG_LEGTRANSFORM_POSES
	// here comes the drawing of poses
	// This adds a large amount of computation by not clearing the list -- not optimal, but not worth fixing at the moment
	
	drawLeftFootPose();
	drawRightFootPose();
	//drawSumPose();

	
#endif
		
		if (_switches->draw_footsteps) {
			if (legchangeflag)
			{
				//std::cout << "LEGCHANGE\n";
				addIsometryPose(collectionindex,_leg_odo->getPrimaryInLocal());
				collectionindex++;
				addIsometryPose(collectionindex,_leg_odo->getPrimaryInLocal());
				collectionindex++;
				_viewer->sendCollection(*_obj, true);
			}
		
		}
	  
		//clock_gettime(CLOCK_REALTIME, &threequat);
		
		PublishEstimatedStates(msg);
    }
	
/*
   clock_gettime(CLOCK_REALTIME, &after);
	long elapsed;
	elapsed = static_cast<long>(mid.tv_nsec) - static_cast<long>(before.tv_nsec);
	double elapsed_us = elapsed/1000.;
	std::cout << "0.50, " << elapsed_us << ", ";// << std::endl;
	
	elapsed = static_cast<long>(threequat.tv_nsec) - static_cast<long>(before.tv_nsec);
	elapsed_us = elapsed/1000.;
	std::cout << "0.75, " << elapsed_us << ", ";// << std::endl;
	
	elapsed = static_cast<long>(after.tv_nsec) - static_cast<long>(before.tv_nsec);
	elapsed_us = elapsed/1000.;
	std::cout << "1.00, " << elapsed_us << std::endl;
	*/

	
	
}

void LegOdometry_Handler::PublishEstimatedStates(const drc::robot_state_t * msg) {
	
	/*
		if (((!pose_initialized_) || (!vo_initialized_))  || (!zheight_initialized_)) {
	    std::cout << "pose or vo or zheight not initialized, refusing to publish EST_ROBOT_STATE\n";
	    return;
	  }
	  */
	
	Eigen::Isometry3d currentPelvis = _leg_odo->getPelvisState();
	Eigen::Vector3d velocity_states = _leg_odo->getPelvisVelocityStates();
	Eigen::Vector3d local_rates     = _leg_odo->getLocalFrameRates();
	
    bot_core::pose_t pose;
    pose.utime  =msg->utime;
    pose.pos[0] =currentPelvis.translation().x();
    pose.pos[1] =currentPelvis.translation().y();
    pose.pos[2] =currentPelvis.translation().z();
    
    Eigen::Quaterniond output_q(currentPelvis.linear());// ?? This was a transpose
    
    // Used this to view the angle estimates, decoupled from the position state estimation
#ifdef PUBLISH_AT_TRUE_POSITION
    std::cout << "Using the true position for the estimated state\n";
    pose.pos[0] = msg->origin_position.translation.x;
    pose.pos[1] = msg->origin_position.translation.y;
    pose.pos[2] = msg->origin_position.translation.z;
#endif
    
    //std::cout << "Output E angles: " << InertialOdometry::QuaternionLib::q2e(output_q).transpose() << std::endl;
    //Eigen::Quaterniond output_q;
    
    // forcing yaw angle for testing of the pitch and roll angle values which is to be published as the estimated state of the robot
      Eigen::Quaterniond dummy_var;
      dummy_var.w() = msg->origin_position.rotation.w;
      dummy_var.x() = msg->origin_position.rotation.x;
      dummy_var.y() = msg->origin_position.rotation.y;
      dummy_var.z() = msg->origin_position.rotation.z;
      // This is to use the true yaw angle
      Eigen::Vector3d E_true = InertialOdometry::QuaternionLib::q2e(dummy_var);
      //InertialOdometry::QuaternionLib::printQuaternion("True quaternion is: ", dummy_var); 
      Eigen::Vector3d E_est = InertialOdometry::QuaternionLib::q2e(output_q);
      //E_true(0) = 0.;
      //E_true(1) = 0.;
      //E_est(2) = 0.;
      // But we are not using the true yaw angle now.. -- this was for testing of the roll and pitch angles, but the problem was found as the transpose of .linear() for Eigen::Isometry3d
      //output_q = InertialOdometry::QuaternionLib::e2q(E_true + E_est);
    
    pose.orientation[0] =output_q.w();
    pose.orientation[1] =output_q.x();
    pose.orientation[2] =output_q.y();
    pose.orientation[3] =output_q.z();
    
    for (int i=0; i<3; i++) {
      pose.vel[i] =velocity_states(i);
      pose.rotation_rate[i] = local_rates(i);
    }
    
    //lcm_->publish("POSE_KIN",&pose);
      lcm_->publish("POSE_BODY" + _channel_extension,&pose);
        
    //Below is copied from Maurice's VO -- publishing of true and estimated robot states
  
  // Infer the Robot's head position from the ground truth root world pose
  bot_core::pose_t pose_msg;
  pose_msg.utime = msg->utime;
  pose_msg.pos[0] = msg->origin_position.translation.x;
  pose_msg.pos[1] = msg->origin_position.translation.y;
  pose_msg.pos[2] = msg->origin_position.translation.z;
  pose_msg.orientation[0] = msg->origin_position.rotation.w;
  pose_msg.orientation[1] = msg->origin_position.rotation.x;
  pose_msg.orientation[2] = msg->origin_position.rotation.y;
  pose_msg.orientation[3] = msg->origin_position.rotation.z;
  lcm_->publish("POSE_BODY_TRUE", &pose_msg);
  
  drc::position_3d_t origin;
  origin.translation.x = currentPelvis.translation().x();
  origin.translation.y = currentPelvis.translation().y();
  origin.translation.z = currentPelvis.translation().z();
  
  origin.rotation.w = output_q.w();
  origin.rotation.x = output_q.x();
  origin.rotation.y = output_q.y();
  origin.rotation.z = output_q.z();  
  
  drc::twist_t twist;
  
  twist.linear_velocity.x = velocity_states(0);
  twist.linear_velocity.y = velocity_states(1);
  twist.linear_velocity.z = velocity_states(2);
//  twist.linear_velocity.x = TRUE_state_msg->origin_twist.linear_velocity.x; //local_to_body_lin_rate_(0);
//  twist.linear_velocity.y = TRUE_state_msg->origin_twist.linear_velocity.y; //local_to_body_lin_rate_(1);
//  twist.linear_velocity.z = TRUE_state_msg->origin_twist.linear_velocity.z; //local_to_body_lin_rate_(2);

  
  
  twist.angular_velocity.x = local_rates(0);
  twist.angular_velocity.y = local_rates(1);
  twist.angular_velocity.z = local_rates(2);
//  twist.angular_velocity.x = TRUE_state_msg->origin_twist.angular_velocity.x;
//  twist.angular_velocity.y = TRUE_state_msg->origin_twist.angular_velocity.y;
//  twist.angular_velocity.z = TRUE_state_msg->origin_twist.angular_velocity.z;
  
  
  // EST is TRUE with sensor estimated position
  drc::robot_state_t msgout;
  msgout = *msg;
  msgout.origin_position = origin;
  msgout.origin_twist = twist;
  
  lcm_->publish("EST_ROBOT_STATE" + _channel_extension, &msgout);
  
  // publish local to head pose
  
  // We have the estimated state data here and the truth data, so think this is the best place push the estimated states to file.
  // new
  int status;
  double matx[16];
  Eigen::Isometry3d body_to_head;
  status = bot_frames_get_trans_mat_4x4_with_utime( _botframes, "head", "body", msg->utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      body_to_head(i,j) = matx[i*4+j];
    }
  }  
  
  // Where is the head at
  
  Eigen::Isometry3d local_to_head; 
  // We are now startin to test this conversion
  if (true) {
	// We think this method may have a problem -- but unsure if it's related to the Isometry * operator, or in the bot body_to_head data.
	
    local_to_head = currentPelvis*body_to_head; // TODO -- not sure if this is corect, must test to confirm
  } else {
	  Eigen::Quaterniond l2body_q(currentPelvis.linear().transpose());// these may be short a transpose operation, as has been the case before
	  Eigen::Quaterniond b2head_q(body_to_head.linear().transpose());
	  Eigen::Isometry3d local_to_head_( InertialOdometry::QuaternionLib::QuaternionProduct_(l2body_q, b2head_q) );
	  local_to_head_.translation() = currentPelvis.translation() + body_to_head.translation();
	  local_to_head = local_to_head_;
	  
  }
  
  
  // now we need the linear and rotational velocity states -- velocity and acclerations are computed wiht the first order differential
  Eigen::Vector3d local_to_head_vel = local_to_head_vel_diff.diff((double)msg->utime*(1E-6), local_to_head.translation());
  Eigen::Vector3d local_to_head_acc = local_to_head_acc_diff.diff((double)msg->utime*(1E-6), local_to_head_vel);
  Eigen::Vector3d local_to_head_rate = local_to_head_rate_diff.diff((double)msg->utime*(1E-6), InertialOdometry::QuaternionLib::C2e(local_to_head.linear()));
  
  // estimate the rotational velocity of the head
    Eigen::Quaterniond l2head_rot(local_to_head.linear()); // this may need that transpose again -- to test TODO
    bot_core::pose_t l2head_msg;
    l2head_msg.utime = msg->utime;
    
    l2head_msg.pos[0] = local_to_head.translation().x();
    l2head_msg.pos[1] = local_to_head.translation().y();
    l2head_msg.pos[2] = local_to_head.translation().z();
    
    l2head_msg.orientation[0] = l2head_rot.w();
    l2head_msg.orientation[1] = l2head_rot.x();
    l2head_msg.orientation[2] = l2head_rot.y();
    l2head_msg.orientation[3] = l2head_rot.z();
    l2head_msg.vel[0]=local_to_head_vel(0);
    l2head_msg.vel[1]=local_to_head_vel(1);
    l2head_msg.vel[2]=local_to_head_vel(2);
    l2head_msg.rotation_rate[0]=local_to_head_rate(0);//is this the correct ordering of the roll pitch yaw
    l2head_msg.rotation_rate[1]=local_to_head_rate(1);// Maurice has it the other way round.. confirm with testing
    l2head_msg.rotation_rate[2]=local_to_head_rate(2);
    l2head_msg.accel[0]=local_to_head_acc(0);
    l2head_msg.accel[1]=local_to_head_acc(1);
    l2head_msg.accel[2]=local_to_head_acc(2);
    lcm_->publish("POSE_HEAD" + _channel_extension, &l2head_msg);  
  
  
  // Logging some some data for analysis in MATLAB
  if (_switches->log_data_files) {
    stringstream ss (stringstream::in | stringstream::out);
	
    ss << msg->origin_position.translation.x << ", " << msg->origin_position.translation.y << ", " << msg->origin_position.translation.z << ", ";
    ss << currentPelvis.translation().x() << ", " << currentPelvis.translation().y() << ", " << currentPelvis.translation().z() << ", ";
    
    ss << msg->origin_twist.linear_velocity.x << ", " << msg->origin_twist.linear_velocity.y << ", " << msg->origin_twist.linear_velocity.z << ", ";
    ss << velocity_states(0) << ", "  << velocity_states(1) << ", "  << velocity_states(2) << ", ";
    
    ss << E_true(0) - E_est(0) << ", ";
    ss << E_true(1) - E_est(1) << ", ";
    ss << E_true(2) - E_est(2) << ", ";
    
    ss << msg->origin_twist.linear_velocity.x - local_rates(0) << ", ";
    ss << msg->origin_twist.linear_velocity.y - local_rates(1) << ", ";
    ss << msg->origin_twist.linear_velocity.z - local_rates(2) << ", ";
    
    
    // The true local to head state is read from an LCM message which is available during devepment
    
    // This is the estimated state and must be compared to the previous
    ss << local_to_head.translation().x() << ", " << local_to_head.translation().y() << ", " << local_to_head.translation().z() << ", ";
    
    
    ss <<std::endl;
    
	state_estimate_error_log << ss.str();
	  
  }
}

void LegOdometry_Handler::torso_imu_handler(	const lcm::ReceiveBuffer* rbuf, 
												const std::string& channel, 
												const  drc::imu_t* msg) {
	
	
	Eigen::Quaterniond q(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);
	Eigen::Vector3d rates_b(msg->angular_velocity[0],msg->angular_velocity[1],msg->angular_velocity[2]);

	/*
	std::cout << "AHRS - q: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl
			  << "E: " << InertialOdometry::QuaternionLib::q2e(q).transpose()
			  << std::endl;
	*/
	
	_leg_odo->setOrientationTransform(q, rates_b);
	
	return;
}

/*
void LegOdometry_Handler::pose_head_true_handler(	const lcm::ReceiveBuffer* rbuf, 
													const std::string& channel, 
													const bot_core_pose_t* msg) {

	
}
*/

void LegOdometry_Handler::PublishFootContactEst(int64_t utime) {
	drc::foot_contact_estimate_t msg_contact_est;
	
	msg_contact_est.utime = utime;
	
	// TODO -- Convert this to use the enumerated types from inside the LCM message
	msg_contact_est.detection_method = DIFF_SCHMITT_WITH_DELAY;
	
	msg_contact_est.left_contact = _leg_odo->leftContactStatus();
	msg_contact_est.right_contact = _leg_odo->rightContactStatus();
	
	lcm_->publish("FOOT_CONTACT_ESTIMATE",&msg_contact_est);
}

void LegOdometry_Handler::drawLeftFootPose() {
	//LinkCollection link(2, std::string("Links"));
	
	//addIsometryPose(_leg_odo->getLeftInLocal());
	
	//addIsometryPose(98, _leg_odo->left_to_pelvis);
	
	//TODO - male left_to_pelvis and other private members in TwoLegOdometry class with get functions of the same name, as is done with Eigen::Isometry3d .translation() and .rotation()
	
	addIsometryPose(97, _leg_odo->left_to_pelvis);
	addIsometryPose(98, _leg_odo->left_to_pelvis);
	
	addIsometryPose(87, _leg_odo->pelvis_to_left);
	addIsometryPose(88, _leg_odo->pelvis_to_left);
	
	//InertialOdometry::QuaternionLib::printEulerAngles("drawLeftFootPose()", _leg_odo->pelvis_to_left);
			
}

void LegOdometry_Handler::drawRightFootPose() {
	//LinkCollection link(2, std::string("Links"));
	
	//addIsometryPose(_leg_odo->getLeftInLocal());
	
	
	addIsometryPose(99,_leg_odo->right_to_pelvis);
	addIsometryPose(100,_leg_odo->right_to_pelvis);
	addIsometryPose(89,_leg_odo->pelvis_to_right);
	addIsometryPose(90,_leg_odo->pelvis_to_right);
	
	//std::cout << "adding right foot pose" << std::endl;
}

void LegOdometry_Handler::drawSumPose() {
	//addIsometryPose(95,_leg_odo->add(_leg_odo->left_to_pelvis,_leg_odo->pelvis_to_right));
	//addIsometryPose(96,_leg_odo->add(_leg_odo->left_to_pelvis,_leg_odo->pelvis_to_right));
	
	addIsometryPose(93,_leg_odo->getSecondaryInLocal());
	addIsometryPose(94,_leg_odo->getSecondaryInLocal());
}


void LegOdometry_Handler::addIsometryPose(int objnumber, const Eigen::Isometry3d &target) {
  // TODO - why are negatives required here
  
  //InertialOdometry::QuaternionLib::printEulerAngles("AddIsometryPose()", target);
  
  Eigen::Vector3d E;
  
  InertialOdometry::QuaternionLib::q2e(Eigen::Quaterniond(target.linear()),E);
  //std::cout << "Going to draw: " << E.transpose() << " @ " << target.translation().transpose() << "\n";
	
  _obj->add(objnumber, isam::Pose3d(target.translation().x(),target.translation().y(),target.translation().z(),E(2),0,0));
	
}

// this function may be depreciated soon
void LegOdometry_Handler::addFootstepPose_draw() {
	std::cout << "Drawing pose for foot: " << (_leg_odo->getActiveFoot() == LEFTFOOT ? "LEFT" : "RIGHT") << std::endl; 
	_obj->add(collectionindex, isam::Pose3d(_leg_odo->getPrimaryInLocal().translation().x(),_leg_odo->getPrimaryInLocal().translation().y(),_leg_odo->getPrimaryInLocal().translation().z(),0,0,0));	
	collectionindex = collectionindex + 1;
}

void LegOdometry_Handler::getTransforms(const drc::robot_state_t * msg, Eigen::Isometry3d &left, Eigen::Isometry3d &right) {
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  
  // 1. Solve for Forward Kinematics:
    _link_tfs.clear();
    
    // call a routine that calculates the transforms the joint_state_t* msg.
    map<string, double> jointpos_in;
    map<string, drc::transform_t > cartpos_out;
    
    for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));
   
    if (!stillbusy)
    {
    	//std::cout << "Trying to solve for Joints to Cartesian\n";
    	stillbusy = true;
    	kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,flatten_tree);
    	stillbusy = false;
    }
    else
    {
    	std::cout << "JntToCart is still busy" << std::endl;
    	// This should generate some type of error or serious warning
    }
    
    //bot_core::rigid_transform_t tf;
    //KDL::Frame T_body_head;
    
    map<string, drc::transform_t >::iterator transform_it_lf;
    map<string, drc::transform_t >::iterator transform_it_rf;
    
    transform_it_lf=cartpos_out.find("l_foot");
    transform_it_rf=cartpos_out.find("r_foot");
    
    //T_body_head = KDL::Frame::Identity();
	  if(transform_it_lf!=cartpos_out.end()){// fk cart pos exists
		// This gives us the translation from body to left foot
#ifdef VERBOSE_DEBUG
	    std::cout << " LEFT: " << transform_it_lf->second.translation.x << ", " << transform_it_lf->second.translation.y << ", " << transform_it_lf->second.translation.z << std::endl;
#endif
	    
	    //std::cout << "ROTATION.x: " << transform_it->second.rotation.x << ", " << transform_it->second.rotation.y << std::endl;
	  }else{
	    std::cout<< "fk position does not exist" <<std::endl;
	  }
	  
	  if(transform_it_lf!=cartpos_out.end()){// fk cart pos exists
#ifdef VERBOSE_DEBUG
  	    std::cout << "RIGHT: " << transform_it_rf->second.translation.x << ", " << transform_it_rf->second.translation.y << ", " << transform_it_rf->second.translation.z << std::endl;
#endif
  	    transform_it_rf->second.rotation;
	  }else{
        std::cout<< "fk position does not exist" << std::endl;
  	  }
    
	  left.translation() << transform_it_lf->second.translation.x, transform_it_lf->second.translation.y, transform_it_lf->second.translation.z;
	  right.translation() << transform_it_rf->second.translation.x, transform_it_rf->second.translation.y, transform_it_rf->second.translation.z;

	  Eigen::Vector3d E_;
	  
	  // quaternion scale and vector ordering seems to be correct
	  Eigen::Quaterniond  leftq(transform_it_lf->second.rotation.w, transform_it_lf->second.rotation.x,transform_it_lf->second.rotation.y,transform_it_lf->second.rotation.z);
	  Eigen::Quaterniond rightq(transform_it_rf->second.rotation.w, transform_it_rf->second.rotation.x,transform_it_rf->second.rotation.y,transform_it_rf->second.rotation.z);
	  
	  //std::cout << "leftq Quaternion values are: " << leftq.w() << ", " << leftq.x() << ", " << leftq.y() << ", " << leftq.z() << std::endl;
	  
	  Eigen::Quaterniond tempq;
	  Eigen::Matrix<double,3,3> leftC, rightC;
	  tempq.setIdentity();
	  
	  //std::cout << ".rotation() is: " << left.rotation() << std::endl;
	  
	  E_ << 0.,0.,0.;
	  InertialOdometry::QuaternionLib::q2e(leftq, E_);
	  //std::cout << "LegOdometry_Handler::getTransforms() leftq 2 E: " << E_.transpose() << std::endl;
	  
	  //leftC = InertialOdometry::QuaternionLib::q2C(leftq);
	  
	  //left.rotate(leftq); // with quaternion
	  //left.rotate(leftC); // with rotation matrix
	  // TODO -- confirm the use of transpose() convert the rotation matrix into the correct frae, as this may be in the q2C function..
	  left.linear() = InertialOdometry::QuaternionLib::q2C(leftq).transpose(); // note Isometry3d.rotation() is still marked as "experimental"
	  //right.rotate(rightq);
	  right.linear() = InertialOdometry::QuaternionLib::q2C(rightq).transpose();
	  
	  
	  //E_<< 0.,0.,0.;
	  //tempq = Eigen::Quaterniond(left.linear().transpose());
	  //std::cout << "left.rotation() Quaternion values are: " << tempq.w() << ", " << tempq.x() << ", " << tempq.y() << ", " << tempq.z() << std::endl;
	  //std::cout << "LegOdometry_Handler::getTransforms() subtracted vals: " << leftq.w() - tempq.w() << ", " << leftq.x() - tempq.x() << ", " << leftq.y() - tempq.y() << ", " << leftq.z() - tempq.z() << std::endl;
	  //InertialOdometry::QuaternionLib::q2e(tempq, E_);
	  //std::cout << "LegOdometry_Handler::getTransforms() tempq 2 E: " << E_.transpose() << std::endl << std::endl;
	  
	  // Matt Antoine says 
	  // myMap.clear();
	  // try sysprof (apt-get install sysprof) system profiler
	  // valgrind (memory leaks, memory bounds checking)
	  //   valgrind --leak-check=full --track-origin=yes --output-fil=path_to_output.txt
}

void LegOdometry_Handler::terminate() {
	std::cout << "Closing and cleaning out LegOdometry_Handler object\n";
	
	_leg_odo->terminate();
}

