#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "RobotStateListener.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_robot_state 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  RobotStateListener::RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm, 
          BotViewer *viewer, int operation_mode):
    _urdf_parsed(false),
    _lcm(lcm),
    _viewer(viewer)
  {
   _last_state_msg_system_timestamp = 0;
   _last_state_msg_sim_timestamp = 0;
   _end_pose_received = false;
   _collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    //lcm ok?
    if(!lcm->good())
    {
      cerr << "\nLCM Not Good: Robot State Handler" << endl;
      return;
    }

    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = lcm->subscribe("ROBOT_MODEL", 
				       &RobotStateListener::handleRobotUrdfMsg,
				       this);    
    _urdf_subscription_on = true;
    //Subscribes to MEAS_JOINT_ANGLES 
    //lcm->subscribe("MEAS_JOINT_ANGLES", &RobotStateListener::handleJointAnglesMsg, this); 
    if (operation_mode==0){
      lcm->subscribe("EST_ROBOT_STATE", &RobotStateListener::handleRobotStateMsg, this); 
    }else if(operation_mode==1){
      lcm->subscribe("EST_ROBOT_STATE_COMPRESSED_LOOPBACK", &RobotStateListener::handleRobotStateMsg, this); 
    }
    lcm->subscribe("CANDIDATE_ROBOT_ENDPOSE", &RobotStateListener::handleCandidateRobotEndPoseMsg, this);   
    
    _jointdof_filter_list.clear();
    _jointdof_filter_list.push_back("l_arm_usy");
    _jointdof_filter_list.push_back("r_arm_usy");
    _jointdof_filter_list.push_back("l_arm_shx");
    _jointdof_filter_list.push_back("r_arm_shx");
    _jointdof_filter_list.push_back("l_arm_ely");
    _jointdof_filter_list.push_back("r_arm_ely");
    _jointdof_filter_list.push_back("l_arm_elx");
    _jointdof_filter_list.push_back("r_arm_elx");
    _jointdof_filter_list.push_back("l_arm_uwy");
    _jointdof_filter_list.push_back("r_arm_uwy");
    _jointdof_filter_list.push_back("l_arm_mwx");
    _jointdof_filter_list.push_back("r_arm_mwx");
    _jointdof_filter_list.push_back("l_leg_hpz");
    _jointdof_filter_list.push_back("r_leg_hpz");
    _jointdof_filter_list.push_back("l_leg_hpx");
    _jointdof_filter_list.push_back("r_leg_hpx");
    _jointdof_filter_list.push_back("l_leg_hpy");
    _jointdof_filter_list.push_back("r_leg_hpy");
    _jointdof_filter_list.push_back("l_leg_kny");
    _jointdof_filter_list.push_back("r_leg_kny");
    _jointdof_filter_list.push_back("l_leg_aky");
    _jointdof_filter_list.push_back("r_leg_aky");
    _jointdof_filter_list.push_back("l_leg_akx");
    _jointdof_filter_list.push_back("r_leg_akx");  
    _jointdof_filter_list.push_back("neck_ay");
    _jointdof_filter_list.push_back("back_bkz");
    _jointdof_filter_list.push_back("back_bky");
    _jointdof_filter_list.push_back("back_bkx");
    
    
    Eigen::Vector3f temp;
    temp << 0,0,0;
    ee_forces_map.insert(make_pair("l_hand", temp));
    ee_forces_map.insert(make_pair("r_hand", temp));
    ee_forces_map.insert(make_pair("l_foot", temp));
    ee_forces_map.insert(make_pair("r_foot", temp));
    ee_torques_map.insert(make_pair("l_hand", temp));
    ee_torques_map.insert(make_pair("r_hand", temp));
    ee_torques_map.insert(make_pair("l_foot", temp));
    ee_torques_map.insert(make_pair("r_foot", temp));
  }

  RobotStateListener::~RobotStateListener() {
     _collision_detector->clear_collision_objects(); 
  }

//-------------------------------------------------------------------------------------      
//=============message callbacks

  //void RobotStateListener::handleJointAnglesMsg(const lcm::ReceiveBuffer* rbuf,
  //						 const string& chan, 
  //						 const drc::joint_angles_t* msg)
  void RobotStateListener::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_state_t* msg)						 
  {
    
    //int64_t tic = bot_timestamp_now();
    if (!_urdf_parsed)
    {
     //cout << msg->utime << endl;
      //cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
      return;
    }
    if(_urdf_subscription_on)
    {			
      cout << "\n handleRobotStateMsg: unsubscribing from _urdf_subscription" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }
    
    // Render at 100Hz of Real Time. Too much rendering will make the viewer less reponsive.
    //cout << msg->utime - _last_state_msg_system_timestamp << endl;
		int64_t now = bot_timestamp_now();//msg->utime
    if(now-_last_state_msg_system_timestamp >= 100000)  // timestamps are in usec
    {
    // cout << now - _last_state_msg_system_timestamp << endl;
    _gl_robot->set_state(*msg);
    
    updateForceAndTorqueCache(msg->force_torque);
    bot_viewer_request_redraw(_viewer);
     _last_state_msg_system_timestamp = now;//msg->utime;
     _last_state_msg_sim_timestamp = msg->utime;
    }

    //int64_t toc = bot_timestamp_now();
    //cout << bot_timestamp_useconds(toc-tic) << endl;
    
  } // end handleMessage
  
  void RobotStateListener::handleCandidateRobotEndPoseMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_state_t* msg)						 
  { 
  
   cout << "\n in handleCandidateRobotEndPoseMsg:" << endl;
    if (!_urdf_parsed)    
      return;   
    if(_urdf_subscription_on)
    {			
      cout << "\n handleCandidateRobotEndPoseMsg: unsubscribing from _urdf_subscription" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }

     _gl_robot->set_future_state(*msg);
     _end_pose_received =true;
     // spawn_pose_approval_dock();
     bot_viewer_request_redraw(_viewer);     
  } // end handleMessage

//-------------------------------------------------------------------------------------        
  void RobotStateListener::handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::robot_urdf_t* msg) 
  {

    if(_urdf_parsed ==false) 
    {
      cout<< "\nurdf handler @ RobotStateListener" << endl;
      // Received robot urdf string. Store it internally and get all available joints.
      _robot_name      = msg->robot_name;
      _urdf_xml_string = msg->urdf_xml_string;
      cout<< "\nReceived urdf_xml_string of robot [" 
      << msg->robot_name << "], storing it internally as a param" << endl;

      bot_gtk_gl_drawing_area_set_context(this->_viewer->gl_area); // Prevents conflict with cam renderer which messes with the gl context
      _gl_robot = shared_ptr<visualization_utils::InteractableGlKinematicBody>(new visualization_utils::InteractableGlKinematicBody(_urdf_xml_string,_collision_detector,true,_robot_name));
  
      cout<< "Number of Joints: " << _gl_robot->get_num_joints() <<endl;
      _gl_robot->set_jointdof_marker_filter(_jointdof_filter_list);
      _gl_robot->disable_joint_limit_enforcement();
      
      //remember that we've parsed the urdf already
      _urdf_parsed = true;
    }
 
  } // end urdf handler

  // Heavily changed when switching to new LCM types. 
  // NB: note that feet forces and torques are 3-axis sensors
  void RobotStateListener::updateForceAndTorqueCache(const drc::force_torque_t &msg) 
  {
     std::map<std::string, Eigen::Vector3f >::iterator it;
     ////////////// Hands ///////////////////////////
     it=ee_forces_map.find("l_hand");
     if(it!=ee_forces_map.end())
     {
       it->second << msg.l_hand_force[0],msg.l_hand_force[1], msg.l_hand_force[2];
     }
     it=ee_torques_map.find("l_hand");
     if(it!=ee_torques_map.end())
     {
       it->second << msg.l_hand_torque[0],msg.l_hand_torque[1], msg.l_hand_torque[2];
     }

     it=ee_forces_map.find("r_hand");
     if(it!=ee_forces_map.end())
     {
       it->second << msg.r_hand_force[0],msg.r_hand_force[1], msg.r_hand_force[2];
     }
     it=ee_torques_map.find("r_hand");
     if(it!=ee_torques_map.end())
     {
       it->second << msg.r_hand_torque[0],msg.r_hand_torque[1], msg.r_hand_torque[2];
     }
     
     ////////////// Feet ////////////////////////////
     it=ee_forces_map.find("l_foot");
     if(it!=ee_forces_map.end())
     {
       it->second << 0,0, msg.l_foot_force_z; 
     }
     it=ee_torques_map.find("l_foot");
     if(it!=ee_torques_map.end())
     {
       it->second << msg.l_foot_torque_x,msg.l_foot_torque_y, 0;
     }

     it=ee_forces_map.find("r_foot");
     if(it!=ee_forces_map.end())
     {
       it->second << 0,0, msg.r_foot_force_z;
     }
     it=ee_torques_map.find("r_foot");
     if(it!=ee_torques_map.end())
     {
       it->second << msg.r_foot_torque_x,msg.r_foot_torque_y, 0;
     }
  } // end updateForceAndTorqueCache
  
  


} //namespace renderer_robot_state


