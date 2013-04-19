
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



using namespace TwoLegs;
using namespace std;

LegOdometry_Handler::LegOdometry_Handler(boost::shared_ptr<lcm::LCM> &lcm_) : _finish(false), lcm_(lcm_) {
	// Create the object we want to use to estimate the robot's pelvis position
	// In this case its a two legged vehicle and we use TwoLegOdometry class for this task
	_leg_odo = new TwoLegOdometry();
	
	if(!lcm_->good())
	  return;
	
	model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
	
	lcm_->subscribe("TRUE_ROBOT_STATE",&LegOdometry_Handler::robot_state_handler,this); 
	
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
	
#if defined( DISPLAY_FOOTSTEP_POSES ) || defined( DRAW_DEBUG_LEGTRANSFORM_POSES )
	_viewer = new Viewer(lcm_viewer);
#endif
	
	return;
}

LegOdometry_Handler::~LegOdometry_Handler() {
	
	//delete model_;
	delete _leg_odo;
	delete _obj;
	delete _viewer;
	delete _link;
	
	lcm_destroy(lcm_viewer); //destroy viewer memory at executable end
	
	
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


//void LegOdometry_Handler::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg) {

void LegOdometry_Handler::robot_state_handler(	const lcm::ReceiveBuffer* rbuf, 
												const std::string& channel, 
												const  drc::robot_state_t* msg) {

	bool legchangeflag;
	
	/*
	std::cout << msg->utime << ", ";
	std::cout << msg->contacts.contact_force[0].z << ", ";
	std::cout << msg->contacts.contact_force[1].z << ", ";*/
	
	//pass left and right leg forces and torques to TwoLegOdometry
	// TODO temporary testing interface
	
	

	Eigen::Isometry3d left;
	Eigen::Isometry3d right;
	
	getTransforms(msg,left,right);
	//InertialOdometry::QuaternionLib::printEulerAngles("after getTransforms()", left);

	
	if (firstpass)
	{
		firstpass = false;
		_leg_odo->ResetWithLeftFootStates(left,right);
		//std::cout << "Footsteps initialized, pelvis at: " << _leg_odo->getPelvisFromStep().translation().transpose() <<"\n";
	}
	_leg_odo->setLegTransforms(left, right);
	
	// TODO -- Check Changed the ordering on 18 April 2013 from (msg->utime, msg->contacts.contact_force[1].z , msg->contacts.contact_force[0].z)
	legchangeflag = _leg_odo->FootLogic(msg->utime, msg->contacts.contact_force[0].z , msg->contacts.contact_force[1].z);
	//returnfootstep = _leg_odo->DetectFootTransistion(msg->utime, msg->contacts.contact_force[1].z , msg->contacts.contact_force[0].z);
	
#ifdef DRAW_DEBUG_LEGTRANSFORM_POSES
	// here comes the drawing of poses
	
	drawLeftFootPose();
	drawRightFootPose();
	//drawSumPose();
	/*
	std::cout << msg->utime << " ";
	addIsometryPose(78, _leg_odo->getPelvisFromStep());
	std::cout << msg->utime << " ";
	addIsometryPose(79, _leg_odo->getPelvisFromStep());
	*/
	//std::cout << "Pelvis from step\n" << _leg_odo->getPelvisFromStep().linear() << std::endl;
	_viewer->sendCollection(*_obj, true);
#endif
	
#ifdef DISPLAY_FOOTSTEP_POSES
	if (legchangeflag)
	{
		//std::cout << "LEGCHANGE\n";
		addIsometryPose(collectionindex,_leg_odo->getPrimaryInLocal());
		collectionindex++;
		addIsometryPose(collectionindex,_leg_odo->getPrimaryInLocal());
		collectionindex++;
	}
	
	_viewer->sendCollection(*_obj, true);
#endif

	/*
	if (_leg_odo->primary_foot() == LEFTFOOT)
		 std::cout << "LEFT  ";// << std:: endl;
	else
		std::cout << "RIGHT ";
	//std::cout << std::endl;
	*/
	//std::cout << _leg_odo->primary_foot() << ", ";
	//std::cout << _leg_odo->getPrimaryInLocal().translation().transpose() << ", ";
	//std::cout << "Pelvis is at   : ";
	//std::cout << currentPelvis.translation().transpose() << ", ";
	
	//std::cout << _leg_odo->pelvis_to_left.translation().transpose() << ", ";
	//std::cout << _leg_odo->left_to_pelvis.translation().transpose() << ", ";
	
	//std::cout << "Secondary is at: " << _leg_odo->getSecondaryInLocal().translation().transpose() << std::endl;
	//std::cout << std::endl;
		Eigen::Isometry3d currentPelvis;
		currentPelvis = _leg_odo->getPelvisFromStep();
        
        bot_core::pose_t pose;
        pose.pos[0] =currentPelvis.translation().x();
        pose.pos[1] =currentPelvis.translation().y();
        pose.pos[2] =currentPelvis.translation().z();
        Eigen::Quaterniond bodykin_q(currentPelvis.linear());
        pose.orientation[0] =bodykin_q.w();
        pose.orientation[1] =bodykin_q.x();
        pose.orientation[2] =bodykin_q.y();
        pose.orientation[3] =bodykin_q.z();
        lcm_->publish("POSE_KIN",&pose);
        

	
        PublishFootContactEst(msg->utime);
}

void LegOdometry_Handler::PublishFootContactEst(int64_t utime) {
	drc::foot_contact_estimate_t msg_contact_est;
	
	msg_contact_est.utime = utime;
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
	  
	  
	  
}

