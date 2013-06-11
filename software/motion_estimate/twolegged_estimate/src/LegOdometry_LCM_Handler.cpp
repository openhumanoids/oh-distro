
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

#include "LegOdometry_LCM_Handler.hpp"
#include "QuaternionLib.h"
#include "data_fusion_parameters.hpp"

using namespace TwoLegs;
using namespace std;

LegOdometry_Handler::LegOdometry_Handler(boost::shared_ptr<lcm::LCM> &lcm_, command_switches* commands):
        _finish(false), lcm_(lcm_) {
	// Create the object we want to use to estimate the robot's pelvis position
	// In this case its a two legged vehicle and we use TwoLegOdometry class for this task
	
	_switches = commands;
	filter_joints_vector_size_set = false;
	
	std::cout << "Switches value for listen to LCM trues is: " << _switches->lcm_read_trues << std::endl;
	
	// There used to be polymorphism here, but that soldier was abandoned for an easier and more cumbersome maneuver
	//for (int i=0;i<FILTER_ARR;i++) {_filter[i] = &lpfilter[i];}
	
	 _botparam = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
	 _botframes= bot_frames_get_global(lcm_->getUnderlyingLCM(), _botparam);
	
	FovisEst.P << NAN,NAN,NAN;
	FovisEst.V << 0.,0.,0.;

	body_to_head.setIdentity();

	first_get_transforms = true;
	zvu_flag = false;
	ratecounter = 0;
	local_to_head_vel_diff.setSize(3);
	local_to_head_acc_diff.setSize(3);
	local_to_head_rate_diff.setSize(3);
	zvu_timetrigger.setParameters(0.4,0.6,50000);
	
	stageA_test_vel.setSize(3);

	rate_changer.setSize(3);
	fusion_rate.setSize(1);

	acc_bias_est.setSize(3);

	rate_changer.setDesiredPeriod_us(0,4500);

	// Tuning parameters for data fusion
	fusion_rate.setDesiredPeriod_us(0,DATA_FUSION_PERIOD-500);
	df_feedback_gain = -0.5;
	df_events = 0;

#ifdef DO_FOOT_SLIP_FEEDBACK
	{
		// TODO -- remove this -- used to subjectively measure the foot velocity
		//Eigen::VectorXd w(5);
		//w << 0.2,0.2,0.2,0.2,0.2;
		//Eigen::VectorXd t(5);
		//t << 5000, 10000, 15000, 20000, 25000;
		SFootPrintOut.setSize(3);
		FootVelCompensation.setSize(3);
	}
#endif


	imu_msg_received = false;

	for (int i=0;i<3;i++) {
		median_filter[i].setLength(_switches->medianlength);
	}

#ifdef LOG_LEG_TRANSFORMS
	for (int i=0;i<4;i++) {
		// left vel, right vel, left rate, right rate
		pelvis_to_feet_speed[i].setSize(3);
	}
	for (int i=0;i<12;i++) {
		pelvis_to_feet_transform[i]=0.; // left vel, right vel, left rate, right rate
	}
#endif

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
	if (_switches->do_estimation) {
		lcm_->subscribe("TORSO_IMU",&LegOdometry_Handler::torso_imu_handler,this);
	}

	lcm_->subscribe("FOVIS_REL_ODOMETRY",&LegOdometry_Handler::delta_vo_handler,this);

	// TODO -- the logging of joint commands was added quickly and is therefore added as a define based inclusion. if this is to stay, then proper dynamic size coding must be done
#ifdef LOG_28_JOINT_COMMANDS
	lcm_->subscribe("JOINT_COMMANDS", &LegOdometry_Handler::joint_commands_handler,this);
#endif
	
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
	_obj_leg_poses = new ObjectCollection(1, std::string("Objects"), VS_OBJ_COLLECTION_T_POSE3D);
	_link = new LinkCollection(2, std::string("Links"));
	
	firstpass = 1;//LowPassFilter::getTapSize(); the filter now initializes internally from the first sample -- only a single outside state from the user is required, i.e. transparent
	
	pulse_counter = 0;

	time_avg_counter = 0;
	elapsed_us = 0;
	maxtime = 0;
	prev_frame_utime = 0;

#ifdef LOG_28_JOINT_COMMANDS
	for (int i=0;i<NUMBER_JOINTS;i++) {
	  joint_commands[i] = 0.;
	}
#endif

	_leg_odo = new TwoLegOdometry(_switches->log_data_files, _switches->publish_footcontact_states);
//#if defined( DISPLAY_FOOTSTEP_POSES ) || defined( DRAW_DEBUG_LEGTRANSFORM_POSES )
  if (_switches->draw_footsteps) {
	_viewer = new Viewer(lcm_viewer);
  }
	//#endif
    state_estimate_error_log.Open(_switches->log_data_files,"true_estimated_states.csv");
	joint_data_log.Open(_switches->log_data_files,"joint_data.csv");
	return;
}

LegOdometry_Handler::~LegOdometry_Handler() {
	
	state_estimate_error_log.Close();
	joint_data_log.Close();

	//delete model_;
	delete _leg_odo;
	delete _obj;
	delete _obj_leg_poses;
	delete _link;
	
	joint_lpfilters.clear();
	
	lcm_destroy(lcm_viewer); //destroy viewer memory at executable end
	delete _viewer;
	
	// Not sure if the pointers must be deleted here, this may try and delete a pointer to a static memory location -- therefore commented
	//delete _botframes;
	//delete _botparam;
	
	cout << "Everything Destroyed in LegOdometry_Handler::~LegOdometry_Handler()" << endl;
	return;
}

void LegOdometry_Handler::InitializeFilters(const int num_filters) {
	
	for (int i=0;i<num_filters;i++) {
		LowPassFilter member;
		joint_lpfilters.push_back(member);
	}
}

// To be moved to a better abstraction location
void LegOdometry_Handler::DetermineLegContactStates(long utime, float left_z, float right_z) {
	// The idea here is to determine the contact state of each foot independently
	// to enable better initialization logic when the robot is stating up, or stading up after falling down
	_leg_odo->updateSingleFootContactStates(utime, left_z, right_z);
}

void LegOdometry_Handler::FilterHandForces(const drc::robot_state_t* msg, drc::robot_state_t* estmsg) {

	Eigen::VectorXd lefthand(6);
	Eigen::VectorXd righthand(6);

	map<string, drc::vector_3d_t> forces;
	map<string, drc::vector_3d_t> torques;
	map<string, int> order;

	for (int i=0;i<msg->contacts.num_contacts;i++) {
		forces.insert(make_pair(msg->contacts.id[i], msg->contacts.contact_force[i]));
		torques.insert(make_pair(msg->contacts.id[i], msg->contacts.contact_torque[i]));
		order.insert(make_pair(msg->contacts.id[i],i));

	}
	typedef map<string, drc::vector_3d_t >  contact_map_type_;
	contact_map_type_::iterator contact_lf,contact_rf,contact_lt,contact_rt;

	contact_lf=forces.find("l_hand");
	contact_rf=forces.find("r_hand");
	contact_lt=torques.find("l_hand");
	contact_rt=torques.find("r_hand");

	lefthand(0) = lefthandforcesfilters[0].processSample(contact_lf->second.x);
	lefthand(1) = lefthandforcesfilters[1].processSample(contact_lf->second.y);
	lefthand(2) = lefthandforcesfilters[2].processSample(contact_lf->second.z);

	lefthand(3) = lefthandforcesfilters[3].processSample(contact_lt->second.x);
	lefthand(4) = lefthandforcesfilters[4].processSample(contact_lt->second.y);
	lefthand(5) = lefthandforcesfilters[5].processSample(contact_lt->second.z);

	righthand(0) =righthandforcesfilters[0].processSample(contact_rf->second.x);
	righthand(1) =righthandforcesfilters[1].processSample(contact_rf->second.y);
	righthand(2) =righthandforcesfilters[2].processSample(contact_rf->second.z);

	righthand(3) =righthandforcesfilters[3].processSample(contact_rt->second.x);
	righthand(4) =righthandforcesfilters[4].processSample(contact_rt->second.y);
	righthand(5) =righthandforcesfilters[5].processSample(contact_rt->second.z);

	//estmsg->contacts = msg->contacts;
	estmsg->contacts.num_contacts = msg->contacts.num_contacts;
	int j=order.find("l_hand")->second;
	estmsg->contacts.contact_force[j].x=lefthand(0);
	estmsg->contacts.contact_force[j].y=lefthand(1);
	estmsg->contacts.contact_force[j].z=lefthand(2);

	estmsg->contacts.contact_torque[j].x=lefthand(3);
	estmsg->contacts.contact_torque[j].y=lefthand(4);
	estmsg->contacts.contact_torque[j].z=lefthand(5);

	j=order.find("r_hand")->second;
	estmsg->contacts.contact_force[j].x=righthand(0);
	estmsg->contacts.contact_force[j].y=righthand(1);
	estmsg->contacts.contact_force[j].z=righthand(2);

	estmsg->contacts.contact_torque[j].x=righthand(3);
	estmsg->contacts.contact_torque[j].y=righthand(4);
	estmsg->contacts.contact_torque[j].z=righthand(5);

}

void LegOdometry_Handler::ParseFootForces(const drc::robot_state_t* msg, double &left_force, double &right_force) {
	// TODO -- This must be updated to use naming and not numerical 0 and 1 for left to right foot isolation
	
#ifdef TRY_FOOT_FORCE_MAP_FIND

	// using a map to find the forces each time is not the most efficient way, but its flexible and useful for when we need to change to using forces from the hands when climbing ladders
	// can optimize here if required, but the overhead for this is expected to be reasonable
	map<string, double> foot_forces;
	for (int i=0;i<msg->contacts.num_contacts;i++) {
		foot_forces.insert(make_pair(msg->contacts.id[i], msg->contact_force[i]));
	}

	map<string, double >::iterator contact_lf;
	map<string, double >::iterator contact_rf;

	contact_lf=cartpos_out.find("l_foot");
	contact_rf=cartpos_out.find("r_foot");

	left_force = lpfilter[0].processSample(contact_lf->second);
	right_force = lpfilter[1].processSample(contact_rf->second);

#else


	left_force  = (double)lpfilter[0].processSample(msg->contacts.contact_force[0].z);
	right_force = (double)lpfilter[1].processSample(msg->contacts.contact_force[1].z);


#endif

}

InertialOdometry::DynamicState LegOdometry_Handler::data_fusion(	const unsigned long long &uts,
																	const InertialOdometry::DynamicState &LeggO,
																	const InertialOdometry::DynamicState &InerO,
																	const InertialOdometry::DynamicState &Fovis) {

	Eigen::VectorXd dummy(1);
	dummy << 0;

	if (fusion_rate.genericRateChange(uts,dummy,dummy)) {
		// Yes now we do comparison and feedback

		df_events++;

		Eigen::Vector3d err_b, errv_b, errv_b_VO;
		Eigen::Matrix3d C_wb;


		// Reset inertial position offset here

		double a;
		double b;

		a = 1/(1 + 1000*err_b.norm());
		b = 1/(1 + 100*errv_b.norm());

		// test for zero velocity
		double speed;

		speed = LeggO.V.norm();

		C_wb = q2C(InerO.q).transpose();

		// Determine the error in the body frame
		err_b = C_wb * (LeggO.P - InerO.P);
		errv_b = C_wb * (LeggO.V - InerO.V);
		errv_b_VO = C_wb * (Fovis.V - InerO.V);


		// We should force velocity back to zero if we are confident that robot is standing still
		// Do that here
		//TODO


		double db_a[3];

		if (true) {
			//std::cout << "double value is: " << uts << " | " << ((speed < ZU_SPEED_THRES)) << ", " << ((double)(speed < ZU_SPEED_THRES)) << std::endl;
			zvu_timetrigger.UpdateState(uts,((double)(speed < ZU_SPEED_THRES)));
			//std::cout << "ST state is: " << zvu_timetrigger.getState() << std::endl;

			for (int i=0;i<3;i++) {
				if (zvu_timetrigger.getState() >= 0.5 ) {
					// This is the zero velocity update step
					//std::cout << "ZVU is happening\n";
					zvu_flag = true;

					db_a[i] = - INS_POS_FEEDBACK_GAIN * err_b(i) - INS_VEL_FEEDBACK_GAIN * errv_b(i);

					feedback_loggings[i] = - INS_POS_FEEDBACK_GAIN * err_b(i);
					feedback_loggings[i+3] = - INS_VEL_FEEDBACK_GAIN * errv_b(i);
				}
				else
				{
					zvu_flag = false;

					db_a[i] = - 0 * INS_POS_FEEDBACK_GAIN * err_b(i) - 0 * INS_VEL_FEEDBACK_GAIN * errv_b(i);

					feedback_loggings[i] = - 0 * INS_POS_FEEDBACK_GAIN * err_b(i);
					feedback_loggings[i+3] = - 0 * INS_VEL_FEEDBACK_GAIN * errv_b(i);
				}

			}



		} else {
			for (int i=0;i<3;i++) {
				db_a[i] = - INS_POS_FEEDBACK_GAIN * err_b(i) - INS_VEL_FEEDBACK_GAIN * errv_b_VO(i);
			}
		}



		// Update the bias estimates
		inert_odo.imu_compensator.AccumulateAccelBiases(db_a);

		inert_odo.setPositionState(INS_POS_WINDUP_BALANCE*InerO.P + (1.-INS_POS_WINDUP_BALANCE)*LeggO.P);

		if (true) {
			inert_odo.setVelocityState(INS_VEL_WINDUP_BALANCE*InerO.V + (1.-INS_VEL_WINDUP_BALANCE)*LeggO.V);
	    } else {
			inert_odo.setVelocityState(INS_VEL_WINDUP_BALANCE*InerO.V + (1.-INS_VEL_WINDUP_BALANCE)*Fovis.V);
		}

		// Dynamic wind up reset
		//inert_odo.setPositionState((0.5+0.4*(a))*InerO.P + (0.4*(1-a)+0.1)*LeggO.P);
		//inert_odo.setVelocityState((0.8+0.1*(b))*InerO.V + (0.1*(1-b)+0.1)*LeggO.V);


		if (_switches->verbose) {
			std::cout << "LeggO: " << LeggO.P.transpose() << std::endl
					  << "InerO: " << InerO.P.transpose() << std::endl
					  << "dP_w : " << (LeggO.P - InerO.P).transpose() << std::endl
					  << "IO.q : " << InerO.q.w() << ", " << InerO.q.x() << ", " << InerO.q.y() << ", " << InerO.q.z() << std::endl
					  << "err_b: " << err_b.transpose() << std::endl
					  << "acc_b: " << just_checking_imu_frame.acc_b.transpose() << std::endl
					  << "b_acc: " << inert_odo.imu_compensator.get_accel_biases().transpose() << std::endl
					  << "acc_c: " << just_checking_imu_frame.acc_comp.transpose() << std::endl
					  << "accel_ " << just_checking_imu_frame.accel_.transpose() << std::endl
					  << "force_ " << just_checking_imu_frame.force_.transpose() << std::endl
					  << "a gain " << err_b.norm() << " | "<< a << ", " << (0.6+0.3*(a)) << ", " << 0.3*(1-a)+0.1 << ", " << (0.6+0.3*(a))+0.3*(1-a)+0.1 << std::endl
					  << "C_w2b: " << std::endl << q2C(InerO.q).transpose() << std::endl
					  << std::endl;
		}

	}

	InertialOdometry::DynamicState retstate;
	retstate = InerO;
	retstate.P = LeggO.P;

	return retstate;

}

void LegOdometry_Handler::robot_state_handler(	const lcm::ReceiveBuffer* rbuf, 
												const std::string& channel, 
												const drc::robot_state_t* _msg) {

	//clock_gettime(CLOCK_REALTIME, &before);
	if (_switches->print_computation_time) {
		gettimeofday(&before,NULL);
	}

	if (_msg->utime - prev_frame_utime > 1000) {
		std::cerr << _msg->utime << " @ " << (_msg->utime - prev_frame_utime)/1000 << " frames were missed\n";
	}

	prev_frame_utime = _msg->utime;

	//std::cout << before.tv_nsec << ", " << spare.tv_nsec << std::endl;
	//spare_time = (before.tv_nsec) - (spare.tv_nsec);


	// The intention is to build up the information inside these messages and pass them out on LCM to who ever needs to consume them
	// The estimated state message variables are created here for two main reasons:
	// 1. Timing of the messaging sending is slaved to the reception of joint angle measurements
	// 2. After an estimation iteration these state variables go out of scope, preventing stale data to be carried over to the next iteration of this code (short of memory errors -- which must NEVER happen)
	// This is the main core memory of the state estimation process. These values are the single point of interface with the LCM cloud -- method of odometry will have their own state memory,
	// and should always be managed as such.
	drc::robot_state_t est_msgout;
	bot_core::pose_t est_headmsg;
	Eigen::Isometry3d left;
	Eigen::Isometry3d right;
	bool legchangeflag;
	
	int joints_were_updated=0;


	double left_force, right_force;


	ParseFootForces(_msg, left_force, right_force);


	DetermineLegContactStates((long)_msg->utime,left_force,right_force); // should we have a separate foot contact state classifier, which is not embedded in the leg odometry estimation process
	if (_switches->publish_footcontact_states) {
	  //std::cout << "Foot forces are: " << left_force << ", " << right_force << std::endl;
	  PublishFootContactEst(_msg->utime);
	}
	
#ifdef TRUE_ROBOT_STATE_MSG_AVAILABLE
	// maintain a true pelvis position for drawing of the foot
	true_pelvis.translation() << _msg->origin_position.translation.x, _msg->origin_position.translation.y, _msg->origin_position.translation.z;

	Eigen::Quaterniond tq(_msg->origin_position.rotation.w, _msg->origin_position.rotation.x, _msg->origin_position.rotation.y, _msg->origin_position.rotation.z);

	//true_pelvis.rotate(tq);
	true_pelvis.linear() = q2C(tq);

	// TODO -- This is to be removed, only using this for testing
	//_leg_odo->setTruthE(InertialOdometry::QuaternionLib::q2e(tq));
	//std::cout << "true check\n";
	_leg_odo->setTruthE(q2e_new(tq));

#endif

	// Here we start populating the estimated robot state data
	// TODO -- Measure the computation time required for this copy operation.
	est_msgout = *_msg;

	FilterHandForces(_msg, &est_msgout);


	int ratechangeiter=0;

	if (_switches->do_estimation){
		
		double alljoints[_msg->num_joints];
		std::string jointnames[_msg->num_joints];
		map<std::string, double> jointpos_in;
		Eigen::Isometry3d current_pelvis;
		Eigen::VectorXd pelvis_velocity(3);

		getJoints(_msg, alljoints, jointnames);
		joints_to_map(alljoints,jointnames, _msg->num_joints, &jointpos_in);
		getTransforms_FK(_msg->utime, jointpos_in, left,right);

		// TODO -- Initialization before the VRC..
		if (firstpass>0)
		{
			firstpass--;// = false;

			if (_switches->grab_true_init) {
				_leg_odo->ResetWithLeftFootStates(left,right,true_pelvis);


			} else {
				Eigen::Isometry3d init_state;
				init_state.setIdentity();
				_leg_odo->ResetWithLeftFootStates(left,right,init_state);
			}
		}

		// This must be broken into separate position and velocity states
		legchangeflag = _leg_odo->UpdateStates(_msg->utime, left, right, left_force, right_force); //footstep propagation happens in here
		current_pelvis = _leg_odo->getPelvisState();


		double pos[3];
		double vel[3];

		if (_switches->OPTION_A) {

			// median filter
			pos[0] = median_filter[0].processSample(current_pelvis.translation().x());
			pos[1] = median_filter[1].processSample(current_pelvis.translation().y());
			pos[2] = median_filter[2].processSample(current_pelvis.translation().z());

			current_pelvis.translation().x() = pos[0];
			current_pelvis.translation().y() = pos[1];
			current_pelvis.translation().z() = pos[2];

			stageA[0] = pos[0];
			stageA[1] = pos[1];
			stageA[2] = pos[2];

			// DD
			_leg_odo->calculateUpdateVelocityStates(_msg->utime, current_pelvis);

			stageB[0] = _leg_odo->getPelvisVelocityStates()(0);
			stageB[1] = _leg_odo->getPelvisVelocityStates()(1);
			stageB[2] = _leg_odo->getPelvisVelocityStates()(2);

			// Rate change
			ratechangeiter = rate_changer.genericRateChange(_msg->utime, _leg_odo->getPelvisVelocityStates(), pelvis_velocity);
			if (ratechangeiter==1) {
				_leg_odo->overwritePelvisVelocity(pelvis_velocity);
			}

			stageC[0] = pelvis_velocity(0);
			stageC[1] = pelvis_velocity(1);
			stageC[2] = pelvis_velocity(2);
		}

		if (_switches->OPTION_B) {

			// Dist Diff
			_leg_odo->calculateUpdateVelocityStates(_msg->utime, current_pelvis);

			stageA[0] = _leg_odo->getPelvisVelocityStates()(0);
			stageA[1] = _leg_odo->getPelvisVelocityStates()(1);
			stageA[2] = _leg_odo->getPelvisVelocityStates()(2);

			// Median
			pelvis_velocity(0) = median_filter[0].processSample(_leg_odo->getPelvisVelocityStates()(0));
			pelvis_velocity(1) = median_filter[1].processSample(_leg_odo->getPelvisVelocityStates()(1));
			pelvis_velocity(2) = median_filter[2].processSample(_leg_odo->getPelvisVelocityStates()(2));

			_leg_odo->overwritePelvisVelocity(pelvis_velocity);

			stageB[0] = pelvis_velocity(0);
			stageB[1] = pelvis_velocity(1);
			stageB[2] = pelvis_velocity(2);


			// Rate change
			ratechangeiter = rate_changer.genericRateChange(_msg->utime, _leg_odo->getPelvisVelocityStates(), pelvis_velocity);
			if (ratechangeiter==1) {
				_leg_odo->overwritePelvisVelocity(pelvis_velocity);
			}
		}

		if (_switches->OPTION_C || _switches->OPTION_D) {

			Eigen::Vector3d store_vel_state;
			Eigen::VectorXd filtered_pelvis_vel(3);

			Eigen::Vector3d pos;
			pos << current_pelvis.translation().x(),current_pelvis.translation().y(),current_pelvis.translation().z();

			store_vel_state = stageA_test_vel.diff(_msg->utime,pos);

			stageA[0] = store_vel_state(0);
			stageA[1] = store_vel_state(1);
			stageA[2] = store_vel_state(2);

			// DD
			_leg_odo->calculateUpdateVelocityStates(_msg->utime, current_pelvis);

			stageB[0] = _leg_odo->getPelvisVelocityStates()(0);
			stageB[1] = _leg_odo->getPelvisVelocityStates()(1);
			stageB[2] = _leg_odo->getPelvisVelocityStates()(2);

			// Rate change

			store_vel_state = _leg_odo->getPelvisVelocityStates();
			ratechangeiter = rate_changer.genericRateChange(_msg->utime,store_vel_state,filtered_pelvis_vel);
			_leg_odo->overwritePelvisVelocity(filtered_pelvis_vel);

			if (ratechangeiter==1) {
				stageC[0] = filtered_pelvis_vel(0);
				stageC[1] = filtered_pelvis_vel(1);
				stageC[2] = filtered_pelvis_vel(2);

				// Median Filter
				vel[0] = median_filter[0].processSample(filtered_pelvis_vel(0));
				vel[1] = median_filter[1].processSample(filtered_pelvis_vel(1));
				vel[2] = median_filter[2].processSample(filtered_pelvis_vel(2));

				for (int i=0;i<3;i++) {
					filtered_pelvis_vel(i) = vel[i];
				}

				_leg_odo->overwritePelvisVelocity(filtered_pelvis_vel);

			}
		}

		// At this point the pelvis position has been found from leg kinematics




		// Timing profile. This is the midway point
		//clock_gettime(CLOCK_REALTIME, &mid);
		gettimeofday(&mid,NULL);
		// Here the rate change is propagated into the rest of the system

		if (ratechangeiter==1) {



			InertialOdometry::DynamicState datafusion_out;

			// TODO
			LeggO.P << current_pelvis.translation().x(),current_pelvis.translation().y(),current_pelvis.translation().z();
			LeggO.V = _leg_odo->getPelvisVelocityStates();


			//LeggO.P << 0.,0.,0.;
			//LeggO.V << 0., 0., 0.;

			datafusion_out = data_fusion(_msg->utime, LeggO, InerOdoEst, FovisEst);

			if (_switches->OPTION_D) {
				// This is for the computations that follow -- not directly leg odometry position
				current_pelvis.translation().x() = datafusion_out.P(0);
				current_pelvis.translation().y() = datafusion_out.P(1);
				current_pelvis.translation().z() = datafusion_out.P(2);

				_leg_odo->overwritePelvisVelocity(datafusion_out.V);
			}

			if (isnan((float)datafusion_out.V(0)) || isnan((float)datafusion_out.V(1)) || isnan((float)datafusion_out.V(2))) {
				std::cout << "LegOdometry_Handler::robot_state_handler -- NAN happened\n";
			}


			//legchangeflag = _leg_odo->UpdateStates(_msg->utime, left, right, left_force, right_force);

			// TODO -- remove the foot velocity measurement
#ifdef DO_FOOT_SLIP_FEEDBACK
			switch (_leg_odo->getActiveFoot()) {
			case LEFTFOOT:

				Eigen::VectorXd difffoot(3);
				Eigen::Vector3d trn;
				trn << _leg_odo->getLeftInLocal().translation().x(), _leg_odo->getLeftInLocal().translation().y(), _leg_odo->getLeftInLocal().translation().z();
				difffoot = SFootPrintOut.diff(_msg->utime, trn);

				//std::cout << "LEFT-FOOT subjective: " << std::fixed << difffoot.transpose() << std::endl;




				break;
			}
#endif

			//clock_gettime(CLOCK_REALTIME, &threequat);

			//std::cout << "Standing on: " << (_leg_odo->getActiveFoot()==LEFTFOOT ? "LEFT" : "RIGHT" ) << std::endl;

			if (imu_msg_received) {
				PublishEstimatedStates(_msg, &est_msgout);
				UpdateHeadStates(&est_msgout, &est_headmsg);
				PublishHeadStateMsgs(&est_headmsg);
				PublishH2B((unsigned long long)_msg->utime, body_to_head.inverse());
			}

			#ifdef TRUE_ROBOT_STATE_MSG_AVAILABLE
			// True state messages will ont be available during the VRC and must be removed accordingly
			PublishPoseBodyTrue(_msg);
			#endif
			#ifdef LOG_28_JOINT_COMMANDS
		   for (int i=0;i<16;i++) {
			   measured_joint_effort[i] = _msg->measured_effort[i];
		   }
			#endif


			if (_switches->log_data_files) {
				LogAllStateData(_msg, &est_msgout);
			}
		}// end of the reduced rate portion

    }//do estimation

	if (_switches->draw_footsteps) {
		DrawDebugPoses(left, right, _leg_odo->getPelvisState(), legchangeflag);
	}
 
   if (_switches->print_computation_time) {
	   //clock_gettime(CLOCK_REALTIME, &after);
	   	gettimeofday(&after,NULL);
	    long long elapsed;
	   	/*elapsed = static_cast<long>(mid.tv_nsec) - static_cast<long>(before.tv_nsec);
	   	elapsed_us = elapsed/1000.;
	   	std::cout << "0.50, " << elapsed_us << ", ";// << std::endl;

	   	elapsed = static_cast<long>(threequat.tv_nsec) - static_cast<long>(before.tv_nsec);
	   	elapsed_us = elapsed/1000.;
	   	std::cout << "0.75, " << elapsed_us << ", ";// << std::endl;
	   	*/

		//elapsed = (long long)(static_cast<long long>(spare.tv_usec) - static_cast<long long>(before.tv_usec));
	   elapsed = (after.tv_usec) - (before.tv_usec);

		if (elapsed<=0) {
			std::cout << "Negative elapsed time\n";

		}

		if (elapsed > maxtime) {
			maxtime = elapsed;
		}

		int time_avg_wind = 100;

		elapsed_us += elapsed;
		spare_us += spare_time;

		time_avg_counter++;
		if (time_avg_counter >= time_avg_wind) {
			elapsed_us = elapsed_us/time_avg_wind;
			spare_us = spare_us/time_avg_wind;
			std::cout << "MAX: " << maxtime << " | AVG computation time: [" << elapsed_us << " us]" << std::endl;//, with [" << spare_us << " us] spare" << std::endl;
			spare_us = 0;
			elapsed_us = 0.;
			time_avg_counter = 0;
		}

		//clock_gettime(CLOCK_REALTIME, &spare);
		gettimeofday(&spare,NULL);
   }


}

void LegOdometry_Handler::DrawDebugPoses(const Eigen::Isometry3d &left, const Eigen::Isometry3d &right, const Eigen::Isometry3d &true_pelvis, const bool &legchangeflag) {

#ifdef DRAW_DEBUG_LEGTRANSFORM_POSES
	// This adds a large amount of computation by not clearing the list -- not optimal, but not worth fixing at the moment

	DrawLegPoses(left, right, true_pelvis);
	// This sendCollection call will be overwritten by the one below -- moved here after testing of the forward kinematics
	_viewer->sendCollection(*_obj_leg_poses, true);
#endif

	if (legchangeflag)
	{
		//std::cout << "LEGCHANGE\n";
		addIsometryPose(collectionindex,_leg_odo->getPrimaryInLocal());
		collectionindex++;
		addIsometryPose(collectionindex,_leg_odo->getPrimaryInLocal());
		collectionindex++;
		_viewer->sendCollection(*_obj, true);
	}

	_viewer->sendCollection(*_obj_leg_poses, true);

}

void LegOdometry_Handler::PublishH2B(const unsigned long long &utime, const Eigen::Isometry3d &h2b) {

	Eigen::Quaterniond h2bq = C2q(h2b.linear());

	bot_core::rigid_transform_t tf;
	tf.utime = utime;
	tf.trans[0] = h2b.translation().x();
	tf.trans[1] = h2b.translation().y();
	tf.trans[2] = h2b.translation().z();

	tf.quat[0] = h2bq.w();
	tf.quat[1] = h2bq.x();
	tf.quat[2] = h2bq.y();
	tf.quat[3] = h2bq.z();

	lcm_->publish("HEAD_TO_BODY" + _channel_extension, &tf);
}

void LegOdometry_Handler::PublishEstimatedStates(const drc::robot_state_t * msg, drc::robot_state_t * est_msgout) {
	
	/*
		if (((!pose_initialized_) || (!vo_initialized_))  || (!zheight_initialized_)) {
	    std::cout << "pose or vo or zheight not initialized, refusing to publish EST_ROBOT_STATE\n";
	    return;
	  }
	  */

	drc::position_3d_t origin;
	drc::twist_t twist;
    Eigen::Quaterniond true_q;
    Eigen::Vector3d E_true;
    Eigen::Vector3d E_est;
    bot_core::pose_t pose;
	
	Eigen::Isometry3d currentPelvis   = _leg_odo->getPelvisState();
	Eigen::Vector3d   velocity_states = _leg_odo->getPelvisVelocityStates();
	Eigen::Vector3d   local_rates     = _leg_odo->getLocalFrameRates();
	
	// estimated orientation 
    Eigen::Quaterniond output_q(currentPelvis.linear()); // This is worth checking again
    
    //std::cout << "CHECKING ROTATIONS: " << 57.29*(_leg_odo->truth_E-InertialOdometry::QuaternionLib::q2e(output_q)).norm() << std::endl;

    true_q.w() = msg->origin_position.rotation.w;
    true_q.x() = msg->origin_position.rotation.x;
    true_q.y() = msg->origin_position.rotation.y;
    true_q.z() = msg->origin_position.rotation.z;

    pose.utime  =msg->utime;
    
    for (int i=0; i<3; i++) {
      pose.vel[i] =velocity_states(i);
      pose.rotation_rate[i] = local_rates(i);
    }
  
  // True or estimated position
  if (_switches->use_true_z) {
	pose.pos[0] = msg->origin_position.translation.x;
	pose.pos[1] = msg->origin_position.translation.y;
	pose.pos[2] = msg->origin_position.translation.z;

	pose.orientation[0] =true_q.w();
	pose.orientation[1] =true_q.x();
	pose.orientation[2] =true_q.y();
	pose.orientation[3] =true_q.z();

	origin.translation.x = msg->origin_position.translation.x;
	origin.translation.y = msg->origin_position.translation.y;
	origin.translation.z = msg->origin_position.translation.z;

	origin.rotation.w = msg->origin_position.rotation.w;
	origin.rotation.x = msg->origin_position.rotation.x;
	origin.rotation.y = msg->origin_position.rotation.y;
	origin.rotation.z = msg->origin_position.rotation.z;

	twist.linear_velocity.x = msg->origin_twist.linear_velocity.x; //local_to_body_lin_rate_(0);
	twist.linear_velocity.y = msg->origin_twist.linear_velocity.y; //local_to_body_lin_rate_(1);
	twist.linear_velocity.z = msg->origin_twist.linear_velocity.z; //local_to_body_lin_rate_(2);

	twist.angular_velocity.x = msg->origin_twist.angular_velocity.x;
	twist.angular_velocity.y = msg->origin_twist.angular_velocity.y;
	twist.angular_velocity.z = msg->origin_twist.angular_velocity.z;
  } else {
	pose.pos[0] =currentPelvis.translation().x();
	pose.pos[1] =currentPelvis.translation().y();
	pose.pos[2] =currentPelvis.translation().z();

	pose.orientation[0] =output_q.w();
	pose.orientation[1] =output_q.x();
	pose.orientation[2] =output_q.y();
	pose.orientation[3] =output_q.z();

	origin.translation.x = currentPelvis.translation().x();
	origin.translation.y = currentPelvis.translation().y();
	origin.translation.z = currentPelvis.translation().z();

	origin.rotation.w = output_q.w();
	origin.rotation.x = output_q.x();
	origin.rotation.y = output_q.y();
	origin.rotation.z = output_q.z();

	twist.linear_velocity.x = velocity_states(0);//msg->origin_twist.linear_velocity.x;//velocity_states(0);
	twist.linear_velocity.y = velocity_states(1);//msg->origin_twist.linear_velocity.y;//velocity_states(1);
	twist.linear_velocity.z = velocity_states(2);

	//Eigen::Vector3d wrates;
	//wrates = InertialOdometry::QuaternionLib::q2C(output_q).transpose()*local_rates;

	twist.angular_velocity.x = local_rates(0);
	twist.angular_velocity.y = local_rates(1);
	twist.angular_velocity.z = local_rates(2);
  }

  // EST is TRUE with sensor estimated position
  
  //msgout = *msg;
  est_msgout->origin_position = origin;
  est_msgout->origin_twist = twist;

  lcm_->publish("EST_ROBOT_STATE" + _channel_extension, est_msgout);
  lcm_->publish("POSE_BODY" + _channel_extension,&pose);

	/*
	// TODO -- remove this pulse train, only for testing
	// now add a 40Hz pulse train to the true robot state
	if (msg->utime*1.E-3 - pulse_time_ >= 2000) {
		pulse_time_ = msg->utime*1.E-3;
		pulse_counter = 16;
	}
	if (pulse_counter>0) {
		origin.translation.x = origin.translation.x + 0.003;
		origin.translation.y = origin.translation.y + 0.003;

		twist.linear_velocity.x = twist.linear_velocity.x + 0.03;
		twist.linear_velocity.y = twist.linear_velocity.y + 0.1;
		pulse_counter--;
	}*/
}

// TODO -- remember that this is to be depreciated
// This function will not be available during the VRC, as the true robot state will not be known
void LegOdometry_Handler::PublishPoseBodyTrue(const drc::robot_state_t * msg) {

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
}

void LegOdometry_Handler::PublishHeadStateMsgs(const bot_core::pose_t * msg) {

  lcm_->publish("POSE_HEAD" + _channel_extension, msg);

  return;
}

void LegOdometry_Handler::UpdateHeadStates(const drc::robot_state_t * msg, bot_core::pose_t * l2head_msg) {

	Eigen::Vector3d local_to_head_vel;
	Eigen::Vector3d local_to_head_acc;
	Eigen::Vector3d local_to_head_rate;

	Eigen::Isometry3d local_to_head;

	// Replace this with FK
	if (false) {
		int status;
		double matx[16];
		status = bot_frames_get_trans_mat_4x4_with_utime( _botframes, "head", "body", msg->utime, matx);
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				body_to_head(i,j) = matx[i*4+j];
			}
		}
	}

	

	//std::cout << body_to_head.translation().transpose() << " is b2h\n";
	Eigen::Isometry3d pelvis;
	Eigen::Quaterniond q(msg->origin_position.rotation.w, msg->origin_position.rotation.x, msg->origin_position.rotation.y, msg->origin_position.rotation.z);
	// TODO -- remember this flag

	pelvis.setIdentity();
	pelvis.translation() << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
	pelvis.linear() = q2C(q);

	  //std::cout << truebody.translation().transpose() << " is tb\n";

	local_to_head = pelvis * body_to_head;
	  //std::cout << local_to_head.translation().transpose() << " is l2h\n\n";

	// now we need the linear and rotational velocity states -- velocity and accelerations are computed wiht the first order differential

	local_to_head_vel = local_to_head_vel_diff.diff(msg->utime, local_to_head.translation());
	local_to_head_acc = local_to_head_acc_diff.diff(msg->utime, local_to_head_vel);
	local_to_head_rate = local_to_head_rate_diff.diff(msg->utime, C2e(local_to_head.linear()));

	// estimate the rotational velocity of the head
	Eigen::Quaterniond l2head_rot;
	l2head_rot = C2q(local_to_head.linear());

	l2head_msg->utime = msg->utime;

	l2head_msg->pos[0] = local_to_head.translation().x();
	l2head_msg->pos[1] = local_to_head.translation().y();
	l2head_msg->pos[2] = local_to_head.translation().z();

	l2head_msg->orientation[0] = l2head_rot.w();
	l2head_msg->orientation[1] = l2head_rot.x();
	l2head_msg->orientation[2] = l2head_rot.y();
	l2head_msg->orientation[3] = l2head_rot.z();
	l2head_msg->vel[0]=local_to_head_vel(0);
	l2head_msg->vel[1]=local_to_head_vel(1);
	l2head_msg->vel[2]=local_to_head_vel(2);
	l2head_msg->rotation_rate[0]=local_to_head_rate(0);//is this the correct ordering of the roll pitch yaw
	l2head_msg->rotation_rate[1]=local_to_head_rate(1);// Maurice has it the other way round.. ypr
	l2head_msg->rotation_rate[2]=local_to_head_rate(2);
	l2head_msg->accel[0]=local_to_head_acc(0);
	l2head_msg->accel[1]=local_to_head_acc(1);
	l2head_msg->accel[2]=local_to_head_acc(2);

	return;
}

void LegOdometry_Handler::LogAllStateData(const drc::robot_state_t * msg, const drc::robot_state_t * est_msgout) {

  // Logging csv file with true and estimated states
  stringstream ss (stringstream::in | stringstream::out);

  // The true states are
  stateMessage_to_stream(msg, ss);
  stateMessage_to_stream(est_msgout, ss);

  {
  Eigen::Vector3d elogged;
  elogged = q2e_new(Eigen::Quaterniond(est_msgout->origin_position.rotation.w, est_msgout->origin_position.rotation.x, est_msgout->origin_position.rotation.y, est_msgout->origin_position.rotation.z));

  //std::cout << "logged: " << (_leg_odo->truth_E - elogged).norm() << std::endl;

  }

  // adding timestamp a bit late, sorry
  ss << msg->utime << ", ";

  // Adding the foot contact forces
  ss << msg->contacts.contact_force[0].z << ", "; // left
  ss << msg->contacts.contact_force[1].z << ", "; // right

  // Active foot is
  ss << (_leg_odo->getActiveFoot() == LEFTFOOT ? "0" : "1") << ", ";

  // The single foot contact states are also written to file for reference -- even though its published by a separate processing using this same class.
  ss << _leg_odo->leftContactStatus() << ", ";
  ss << _leg_odo->rightContactStatus() << ", "; // 30

	#ifdef LOG_28_JOINT_COMMANDS
		for (int i=0;i<NUMBER_JOINTS;i++) {
		  ss << joint_commands[i] << ", "; //31-58
		}

		for (int i=0;i<16;i++) {
			ss << joint_positions[i] << ", "; //59-74
		}

	   for (int i=0;i<16;i++) {
		   ss << measured_joint_effort[i] << ", ";//75-90
	   }
	#endif
	#ifdef LOG_LEG_TRANSFORMS
	   // left vel, right vel, left rate, right rate
	   for (int i=0;i<12;i++) {
		   ss << pelvis_to_feet_transform[i] << ", ";//91-102
	   }
	#endif

   for (int i=0;i<16;i++) {
	 ss << filtered_joints[i] << ", "; //103-118
   }

   //std::cout << "Stage A: ";
   for (int i=0;i<3;i++) {
	 ss << stageA[i] << ", "; //119-121
	 //std::cout << stageA[i] << ", ";
   }
   //std::cout << std::endl;
   for (int i=0;i<3;i++) {
	 ss << stageB[i] << ", "; //122-124
   }
   for (int i=0;i<3;i++) {
	 ss << stageC[i] << ", "; //125-127
   }

   for (int i=0;i<3;i++) {
	   ss << InerOdoEst.P(i) << ", "; //128-130
   }
   for (int i=0;i<3;i++) {
	   ss << InerOdoEst.V(i) << ", ";//131-133
   }

   for (int i=0;i<3;i++) {
   	   ss << LeggO.V(i) << ", ";//134-136
   }

   for (int i=0;i<3;i++) {
	   ss << FovisEst.V(i) << ", ";//137-139
   }

   Eigen::Vector3d biasesa;

   biasesa = inert_odo.imu_compensator.get_accel_biases();

   for (int i=0;i<3;i++) {
   	   ss << biasesa(i) << ", ";//140-142
   }

   for (int i=0;i<6;i++) {
       ss << feedback_loggings[i] << ", "; //143-148
   }

   for (int i=0;i<3;i++) {
	  ss << just_checking_imu_frame.force_(i) << ", "; //149-151
   }

   ss << zvu_flag << ", "; //152

   ss <<std::endl;

   state_estimate_error_log << ss.str();

}

// Push the state values in a drc::robot_state_t message type to the given stringstream
void LegOdometry_Handler::stateMessage_to_stream(	const drc::robot_state_t *msg,
													stringstream &ss) {

	Eigen::Quaterniond q(msg->origin_position.rotation.w, msg->origin_position.rotation.x, msg->origin_position.rotation.y, msg->origin_position.rotation.z);
	Eigen::Vector3d E;

	E = q2e_new(q);

	ss << msg->origin_position.translation.x << ", ";
	ss << msg->origin_position.translation.y << ", ";
	ss << msg->origin_position.translation.z << ", ";

	ss << msg->origin_twist.linear_velocity.x << ", ";
	ss << msg->origin_twist.linear_velocity.y << ", ";
	ss << msg->origin_twist.linear_velocity.z << ", ";

	ss << E(0) << ", ";
	ss << E(1) << ", ";
	ss << E(2) << ", ";

	ss << msg->origin_twist.angular_velocity.x << ", ";
	ss << msg->origin_twist.angular_velocity.y << ", ";
	ss << msg->origin_twist.angular_velocity.z << ", ";

	return;
}

void LegOdometry_Handler::torso_imu_handler(	const lcm::ReceiveBuffer* rbuf, 
												const std::string& channel, 
												const  drc::imu_t* msg) {
	


	double rates[3];
	double accels[3];
	double angles[3];

	if (isnan((float)msg->angular_velocity[0]) || isnan((float)msg->angular_velocity[1]) || isnan((float)msg->angular_velocity[2]) || isnan((float)msg->linear_acceleration[0]) || isnan((float)msg->linear_acceleration[1]) || isnan((float)msg->linear_acceleration[2])) {
		std::cerr << "torso_imu_handler -- NAN encountered, skipping this frame\n";
		return;
	}

	Eigen::Quaterniond q(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);

	if (q.norm() <= 0.95) {
		std::cerr << "LegOdometry_Handler::torso_imu_handler -- Non unit quaternion encountered, skipping this frame.\n";
		return;
	}
	
	// To filter or not to filter the angular rates
	if (false) {
		Eigen::Vector3d E;
		E = q2e_new(q);// change to new

		for (int i=0;i<3;i++) {
		  rates[i] = lpfilter[i+2].processSample(msg->angular_velocity[i]); // +2 since the foot force values use the first two filters
		  angles[i] = lpfilter[i+5].processSample(E(i));
		  q = e2q(E);
		}
	} else {
		for (int i=0;i<3;i++) {
			rates[i] = msg->angular_velocity[i]; // +2 since the foot force values use the first two filters
			accels[i] = msg->linear_acceleration[i];
		}
	}
	


	Eigen::Vector3d rates_b(rates[0], rates[1], rates[2]);

	_leg_odo->setOrientationTransform(q, rates_b);

	InertialOdometry::IMU_dataframe imu_data;

	imu_data.uts = msg->utime;


	imu_data.acc_b = Eigen::Vector3d(accels[0],accels[1],accels[2]);


	//Eigen::Quaterniond trivial_OL_q;

	//trivial_OL_q.setIdentity();

	//trivial_OL_q = e2q(Eigen::Vector3d(PI/2*0.,-PI/4., 0.*PI/2));

	//imu_data.acc_b << 0.1+9.81/sqrt(2), 0., +9.81/sqrt(2);

	just_checking_imu_frame = imu_data;
	InerOdoEst = inert_odo.PropagatePrediction(&imu_data,q);

	//std::cout << "T_OL, ut: " << imu_data.uts << ", " << InerOdoEst.V.transpose() << " | " << InerOdoEst.P.transpose() << std::endl;

	if (!imu_msg_received) {
		imu_msg_received = true;
	}

	return;
}

#ifdef LOG_28_JOINT_COMMANDS
void LegOdometry_Handler::joint_commands_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::joint_command_t* msg) {
	// TODO -- 28 actuated joints are hard coded, this must be changed -- but subject to the LOG_28_JOINT_COMMANDS define for the time being
	for (int i=0;i<NUMBER_JOINTS;i++) {
	  joint_commands[i] = msg->effort[i];
	}
}
#endif

/*
void LegOdometry_Handler::pose_head_true_handler(	const lcm::ReceiveBuffer* rbuf, 
													const std::string& channel, 
													const bot_core_pose_t* msg) {

	
}
*/

void LegOdometry_Handler::delta_vo_handler(	const lcm::ReceiveBuffer* rbuf,
												const std::string& channel,
												const fovis::update_t* _msg) {


  //std::cout << "LegOdometry_Handler::delta_vo_handler is happening\n";
  Eigen::Isometry3d motion_estimate;
  Eigen::Vector3d vo_dtrans;

  // This is still in the left camera frame and must be rotated to the pelvis frame
  // we are gokng to do this with the forward kinematics
  // we need camera to head -- this is going to go into a struct
  // (all bot transforms calculated here should be moved here for better abstraction in the future)

  vo_dtrans = bottransforms.lcam2pelvis( Eigen::Vector3d(_msg->translation[0],_msg->translation[1],_msg->translation[2]));

  // here we need calculate velocity perceived by fovis
  // using the time stamps of the messages and just doing the first order difference on the values


  // need the delta translation in the world frame
  FovisEst.V = (inert_odo.C_bw()*vo_dtrans)/((_msg->timestamp - FovisEst.uts)*1E-6); // convert to a world frame velocity

  FovisEst.uts = _msg->timestamp;

}

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
  
  Eigen::Vector3d E;
  E = q2e_new(Eigen::Quaterniond(target.linear()));//change to new
  _obj->add(objnumber, isam::Pose3d(target.translation().x(),target.translation().y(),target.translation().z(),E(2),E(1),E(0)));
}

// Four Isometries must be passed -- representing pelvisto foot and and foot to pelvis transforms
void LegOdometry_Handler::DrawLegPoses(const Eigen::Isometry3d &left, const Eigen::Isometry3d &right, const Eigen::Isometry3d &true_pelvis) {
  
	Eigen::Isometry3d target[4];

	target[0] = left;
	target[1] = right;
	target[2] = left.inverse();
	target[3] = right.inverse();

  Eigen::Vector3d E;
  Eigen::Isometry3d added_vals[2];
  
  Eigen::Isometry3d back_from_feet[2];
  
  //clear the list to prevent memory growth
  _obj_leg_poses->clear();
	

  // The best way to add two isometries -- maintained to be consistent with the rest of the code. This should probably be changed
  for (int i=0;i<2;i++) {
    added_vals[i] = _leg_odo->add(true_pelvis, target[i]); // this is the same function that is used by TwoLegOdometry to accumulate Isometry transforms
    E = q2e_new(Eigen::Quaterniond(added_vals[i].linear()));
    _obj_leg_poses->add(50+i, isam::Pose3d(added_vals[i].translation().x(),added_vals[i].translation().y(),added_vals[i].translation().z(),E(2),E(1),E(0)));
    
    back_from_feet[i] = _leg_odo->add(added_vals[i], target[i+2]);
    E = q2e_new(Eigen::Quaterniond(back_from_feet[i].linear()));
    _obj_leg_poses->add(50+i+2, isam::Pose3d(back_from_feet[i].translation().x(),back_from_feet[i].translation().y(),back_from_feet[i].translation().z(),E(2),E(1),E(0)));
  }
  
}

// this function may be depreciated soon
void LegOdometry_Handler::addFootstepPose_draw() {
	std::cout << "Drawing pose for foot: " << (_leg_odo->getActiveFoot() == LEFTFOOT ? "LEFT" : "RIGHT") << std::endl; 
	_obj->add(collectionindex, isam::Pose3d(_leg_odo->getPrimaryInLocal().translation().x(),_leg_odo->getPrimaryInLocal().translation().y(),_leg_odo->getPrimaryInLocal().translation().z(),0,0,0));	
	collectionindex = collectionindex + 1;
}

void LegOdometry_Handler::getJoints(const drc::robot_state_t * msg, double alljoints[], std::string joint_name[]) {
  
  if (filtered_joints.capacity() != msg->num_joints || !filter_joints_vector_size_set) {
	  std::cout << "LegOdometry_Handler::getJoints -- Automatically changing the capacity of filt.joints -- capacity: " << filtered_joints.capacity() << ", num_joints: " << msg->num_joints << std::endl;
	  filter_joints_vector_size_set = true;
	  filtered_joints.resize(msg->num_joints);
  }



	// call a routine that calculates the transforms the joint_state_t* msg.

	if (first_get_transforms) {
		first_get_transforms = false;
		InitializeFilters((int)msg->num_joints);
	}

	for (uint i=0; i< (uint) msg->num_joints; i++) {
		// Keep joint positions in local memory
		alljoints[i] = msg->joint_position[i];
		joint_name[i] = msg->joint_name[i];

		if (i<16) {
		  joint_positions[i] = msg->joint_position[i];
		}

	}

	//filterJointPositions(msg->utime, msg->num_joints, alljoints);
}

void LegOdometry_Handler::joints_to_map(const double joints[], const std::string joint_name[], const int &num_joints, std::map<string, double> *_jointpos_in) {

	for (uint i=0; i< num_joints; i++) { //cast to uint to suppress compiler warning

	  switch (2) {
		  case 1:
			_jointpos_in->insert(make_pair(joint_name[i], joint_lpfilters.at(i).processSample(joints[i])));
			break;
		  case 2:
			// not using filters on the joint position measurements
			_jointpos_in->insert(make_pair(joint_name[i], joints[i]));//skipping the filters
			break;
		  //case 3:
			//_jointpos_in->insert(make_pair(msg->joint_name[i], filtered_joints[i])); // The rate has been reduced to sample periods greater than 4500us and filtered with integral/rate/diff
			//break;
	  }
	}
}


// TODO -- make a transform to Eigen converter, since this function now has 4 similar conversions
void LegOdometry_Handler::getTransforms_FK(const unsigned long long &u_ts, const map<string, double> &jointpos_in, Eigen::Isometry3d &left, Eigen::Isometry3d &right) {



	bool kinematics_status;
	bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.

	// 1. Solve for Forward Kinematics:
	_link_tfs.clear(); // TODO -- not sure why we have this, to investigate

	map<string, drc::transform_t > cartpos_out;

	/*
	 * map<string, double> _jointpos_in;
	_jointpos_in = jointpos_in;
	for (map<string, double >::iterator it=_jointpos_in.begin();it!=_jointpos_in.end();it++) {
		it->second=0;
	}*/

	kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,flatten_tree);



	map<string, drc::transform_t >::iterator transform_it_ll_l;
	map<string, drc::transform_t >::iterator transform_it_ll_r;

	// This should be depreciated
	transform_it_ll_l=cartpos_out.find("l_talus");
	transform_it_ll_r=cartpos_out.find("r_talus");

	// Find the pelvis to head transform
	map<string, drc::transform_t >::iterator transform_it_ph;
	transform_it_ph=cartpos_out.find("head");

	Eigen::Quaterniond b2head_q(transform_it_ph->second.rotation.w, transform_it_ph->second.rotation.x,transform_it_ph->second.rotation.y,transform_it_ph->second.rotation.z);

	body_to_head.linear() = q2C(b2head_q);
	body_to_head.translation() << transform_it_ph->second.translation.x, transform_it_ph->second.translation.y, transform_it_ph->second.translation.z;

	// get lcam to pelvis transform
	map<string, drc::transform_t >::iterator transform_it_lcam;
	transform_it_lcam=cartpos_out.find("left_camera_optical_frame");

	Eigen::Isometry3d p2lc;
	p2lc.setIdentity();
	p2lc.translation() << transform_it_lcam->second.translation.x, transform_it_lcam->second.translation.y, transform_it_lcam->second.translation.z;
	p2lc.linear() = q2C(Eigen::Quaterniond(transform_it_lcam->second.rotation.w,transform_it_lcam->second.rotation.x,transform_it_lcam->second.rotation.y,transform_it_lcam->second.rotation.z));
	bottransforms.setLCam2Pelvis(p2lc);

	//T_body_head = KDL::Frame::Identity();
	if(transform_it_ll_l!=cartpos_out.end()){// fk cart pos exists
		// This gives us the translation from body to left foot

	}else{
		std::cout<< "fk position does not exist" <<std::endl;
	}

	if(transform_it_ll_r!=cartpos_out.end()){// fk cart pos exists

	}else{
		std::cout<< "fk position does not exist" << std::endl;
	}


	// Get lower leg quaternions
	Eigen::Quaterniond q_ll_l(transform_it_ll_l->second.rotation.w, transform_it_ll_l->second.rotation.x,transform_it_ll_l->second.rotation.y,transform_it_ll_l->second.rotation.z);
	Eigen::Quaterniond q_ll_r(transform_it_ll_r->second.rotation.w, transform_it_ll_r->second.rotation.x,transform_it_ll_r->second.rotation.y,transform_it_ll_r->second.rotation.z);

	Eigen::Isometry3d left_lleg;
	Eigen::Isometry3d right_lleg;

	left_lleg.translation() << transform_it_ll_l->second.translation.x, transform_it_ll_l->second.translation.y, transform_it_ll_l->second.translation.z;
	right_lleg.translation() << transform_it_ll_r->second.translation.x, transform_it_ll_r->second.translation.y, transform_it_ll_r->second.translation.z;

	left_lleg.rotate(q_ll_l);// DO NOT TRUST THIS IN THE DRC CONTEXT
	right_lleg.rotate(q_ll_r);// DO NOT TRUST THIS IN THE DRC CONTEXT

	left_lleg.linear() = q2C(q_ll_l);
	right_lleg.linear() = q2C(q_ll_r);


	// Now i need to imitate the ankle pitch and roll angles
	// this involves rotating the lower leg position with the IMU world angles. pay attention to the order in which these rotations are applied.

	//bot_core::rigid_transform_t tf;
	//KDL::Frame T_body_head;

	map<string, drc::transform_t >::iterator transform_it_lf;
	map<string, drc::transform_t >::iterator transform_it_rf;

	transform_it_lf=cartpos_out.find("l_foot");
	transform_it_rf=cartpos_out.find("r_foot");

	//T_body_head = KDL::Frame::Identity();
	if(transform_it_lf!=cartpos_out.end()){// fk cart pos exists
		// This gives us the translation from body to left foot

	}else{
		std::cout<< "fk position does not exist" <<std::endl;
	}

	if(transform_it_lf!=cartpos_out.end()){// fk cart pos exists

	}else{
		std::cout<< "fk position does not exist" << std::endl;
	}

	//Eigen::Vector3d E_;
	// quaternion scale and vector ordering seems to be correct
	Eigen::Quaterniond  leftq(transform_it_lf->second.rotation.w, transform_it_lf->second.rotation.x,transform_it_lf->second.rotation.y,transform_it_lf->second.rotation.z);
	Eigen::Quaterniond rightq(transform_it_rf->second.rotation.w, transform_it_rf->second.rotation.x,transform_it_rf->second.rotation.y,transform_it_rf->second.rotation.z);
	  

	  //Eigen::Quaterniond tempq;
	  //Eigen::Matrix<double,3,3> leftC, rightC;
	  //tempq.setIdentity();
	  
	  // TODO -- clear this if your are happy with it working
	  if (false) {
		  // TODO -- Depreciate this soon -- lack of trust in Isometry3d::rotate(), many hours lost to this function -- may be a successive vs fixed frame rotation scheme
		  left.setIdentity();
		  right.setIdentity();

		  left.translation() << transform_it_lf->second.translation.x, transform_it_lf->second.translation.y, transform_it_lf->second.translation.z;
		  right.translation() << transform_it_rf->second.translation.x, transform_it_rf->second.translation.y, transform_it_rf->second.translation.z;

		  left.rotate(leftq); // with quaternion
		  right.rotate(rightq);

		  //left.rotate(leftC); // with rotation matrix
	  } else {

		  left.translation() << transform_it_lf->second.translation.x, transform_it_lf->second.translation.y, transform_it_lf->second.translation.z;
		  right.translation() << transform_it_rf->second.translation.x, transform_it_rf->second.translation.y, transform_it_rf->second.translation.z;

		  // TODO -- confirm the use of transpose() convert the rotation matrix into the correct frae, as this may be in the q2C function..
		  //left.linear() = InertialOdometry::QuaternionLib::q2C(leftq).transpose(); // note Isometry3d.rotation() is still marked as "experimental"
		  //right.linear() = InertialOdometry::QuaternionLib::q2C(rightq).transpose();
		  left.linear() = q2C(leftq); // note Isometry3d.rotation() is still marked as "experimental"
		  right.linear() = q2C(rightq);

	  }

	  // level out foot position from IMU
	  Eigen::Isometry3d IMU_rp;
	  IMU_rp.setIdentity();
	  Eigen::Vector3d imu_E;

	  //std::cout << "Bangles: " <<_leg_odo->getLocalOrientation().w() << ", " <<_leg_odo->getLocalOrientation().x() << ", " <<_leg_odo->getLocalOrientation().y() << ", " <<_leg_odo->getLocalOrientation().z() << std::endl;
	  imu_E = q2e_new(_leg_odo->getLocalOrientation());
	  imu_E(2) = 0.;
	  //std::cout << "Aangles: " << imu_E.transpose() << std::endl << q2C(e2q(imu_E)) << std::endl;


	  IMU_rp.linear() = e2C(imu_E);

	  Eigen::Isometry3d temptransform;
	  Eigen::Isometry3d tempright, templeft;


	  temptransform.setIdentity();
	  temptransform = (IMU_rp)*left;
	  left = temptransform;
	  temptransform = (IMU_rp)*right;
	  right = temptransform;



	  // now we strip out the influence of the ankle joints.
	  // We do not need to know the slope of the terrain. Assuming all footsteps are flat at the contact point
	  Eigen::Vector3d stripRP;

	  //stripRP = q2e_new(InertialOdometry::QuaternionLib::C2q(left.linear()));
	  stripRP = q2e_new(C2q(left.linear()));
	  //std::cout << "Stripping left angles: " << stripRP.transpose() << std::endl;
	  stripRP(0) = 0.;
	  stripRP(1) = 0.;

	  templeft.setIdentity();
	  templeft.translation() = left.translation();
	  templeft.linear() = e2C(stripRP);


	  stripRP = q2e_new(C2q(right.linear()));
	  stripRP(0) = 0.;
	  stripRP(1) = 0.;

	  tempright.setIdentity();
	  tempright.translation() = right.translation();
	  tempright.linear() = e2C(stripRP);

	  left = templeft;
	  right = tempright;


	  //std::cout << "LEFT: " << left.translation().transpose() << " | RIGHT " << right.translation().transpose() << "\n";
	  //std::cout << std::endl;



#ifdef LOG_LEG_TRANSFORMS
	  // The idea here is to push all the required data to a single array [pelvis_to_feet_transform], which is to be logged in publish state method

	  Eigen::Vector3d tempvar;
	  int i;

	  tempvar = pelvis_to_feet_speed[0].diff(u_ts,left.translation());
	  // left vel, right vel, left rate, right rate
	  for (i=0;i<3;i++) {
		  pelvis_to_feet_transform[i] = tempvar(i); // left vel, right vel, left rate, right rate
	  }
	  tempvar = pelvis_to_feet_speed[1].diff(u_ts,right.translation());
	  // left vel, right vel, left rate, right rate
	  for (i=0;i<3;i++) {
		  pelvis_to_feet_transform[3+i] = tempvar(i); // left vel, right vel, left rate, right rate
	  }
	  tempvar = pelvis_to_feet_speed[2].diff(u_ts,C2e(left.rotation()));
	  // left vel, right vel, left rate, right rate
	  for (i=0;i<3;i++) {
		  pelvis_to_feet_transform[6+i] = tempvar(i); // left vel, right vel, left rate, right rate
	  }
	  tempvar = pelvis_to_feet_speed[3].diff(u_ts,C2e(right.rotation()));
	  // left vel, right vel, left rate, right rate
	  for (i=0;i<3;i++) {
		  pelvis_to_feet_transform[9+i] = tempvar(i); // left vel, right vel, left rate, right rate
	  }

#endif

	  // Matt A says
	  // try sysprof (apt-get install sysprof) system profiler
	  // valgrind (memory leaks, memory bounds checking)
	  // valgrind --leak-check=full --track-origin=yes --output-fil=path_to_output.txt
	  

}

// This member function is no longer in use
int LegOdometry_Handler::filterJointPositions(const unsigned long long &ts, const int &num_joints, double alljoints[]) {


	int returnval=0;
	/*Eigen::VectorXd int_vals(num_joints);
	Eigen::VectorXd diff_vals(num_joints);

	//std::cout << " | " << std::fixed << alljoints[5];
	// Integrate to lose noise, but keep information
	int_vals = joint_integrator.integrate(ts, num_joints, alljoints);
	//std::cout << " i: " << int_vals(5);


	// we are looking for a 200Hz process -- 5ms
	if (rate_changer.checkNewRateTrigger(ts)) {
		diff_vals = joint_pos_filter.diff(ts, int_vals);
		//joint_integrator.reset_in_time();

		for (int i=0;i<num_joints;i++) {
			filtered_joints[i] = diff_vals(i);
		}

		std::cout << ", after: " << filtered_joints[5] << "\n";
		returnval = 1;
	}
	*/
	return returnval;
}

void LegOdometry_Handler::terminate() {
	std::cout << "Closing and cleaning out LegOdometry_Handler object\n";
	
	_leg_odo->terminate();
}

Eigen::Vector3d LegOdometry_Handler::getInerAccBiases() {

	return inert_odo.imu_compensator.get_accel_biases();
}

