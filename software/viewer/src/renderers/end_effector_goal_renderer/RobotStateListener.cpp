#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "RobotStateListener.hpp"

using namespace std;
using namespace boost;

namespace ee_goal_renderer 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  RobotStateListener::RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer):
    _lcm(lcm),
    _viewer(viewer)

  {
    //lcm ok?
    if(!lcm->good())
      {
	cerr << "\nLCM Not Good: Robot State Handler" << endl;
	return;
      }

    T_body_world = KDL::Frame::Identity();
    cartpos_out.clear();
    cartpos_out_utime =0;

   //TODO: Must also sunscribe to URDF and parse a KDL tree for FK
    _urdf_subscription = lcm->subscribe("ROBOT_MODEL", 
				       &ee_goal_renderer::RobotStateListener::handleRobotUrdfMsg,
				       this);    
    _urdf_subscription_on = true;

    // Subscribe to Robot_state. 
    lcm->subscribe("EST_ROBOT_STATE", &ee_goal_renderer::RobotStateListener::handleRobotStateMsg, this); 

  }
  
  RobotStateListener::~RobotStateListener() {}


  //=============message callbacks

void RobotStateListener::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_state_t* msg)						 
  { 

   if(!_urdf_parsed)
      {
	//cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
	return;
      }
    if(_urdf_subscription_on)
     {
       cout << "\n handleRobotStateMsg: unsubscribing from _urdf_subscription" << endl;
       _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
	 _urdf_subscription_on =  false; 	
    }

      KDL::Frame  T_world_body;
  	    
      T_world_body.p[0]= msg->origin_position.translation.x;
	    T_world_body.p[1]= msg->origin_position.translation.y;
	    T_world_body.p[2]= msg->origin_position.translation.z;		    
	    T_world_body.M =  KDL::Rotation::Quaternion(msg->origin_position.rotation.x, msg->origin_position.rotation.y, msg->origin_position.rotation.z, msg->origin_position.rotation.w);

      T_body_world=T_world_body.Inverse();   


    // Perform FK to get Transforms between root link for arm control (for mit_drc_robot its utorso - parent link where shoulder joint connect) and body origin (pelvis)
    std::map<std::string, double> jointpos_in;
    for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));



    // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin.
    //Otherwise returns relative transforms between joints.

    kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
    cartpos_out_utime = msg->utime;

    if(kinematics_status>=0){
      // cout << "Success!" <<endl;
    }else{
      std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
      return;
    }
    
  } // end handleMessage


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
    
    // Get a urdf Model from the xml string and get all the joint names.
    urdf::Model robot_model; 
    if (!robot_model.initString( msg->urdf_xml_string))
      {
       cerr << "ERROR: Could not generate robot model" << endl;
     }

   _links_map =  robot_model.links_;

   //---------parse the tree and stop listening for urdf messages

   // Parse KDL tree
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
      {
	cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl; 
	return;
      }

    //unsubscribe from urdf messages
    //_lcm->unsubscribe(_urdf_subscription);  // crashes viewer if there are other urdf subscriptions in other renderers.
    
    //
    _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
    //remember that we've parsed the urdf already
    _urdf_parsed = true;

    
    }//  if(_urdf_parsed ==false)   
  } 

  //bool RobotStateListener::getLinkTf(std::string link_name, bot_core::rigid_transform_t &link_tf)
  bool RobotStateListener::getLinkTf(std::string link_name,  KDL::Frame &T_body_link)
  {
    if (!_urdf_parsed)
    {
      std::cerr<< "URDF not parsed yet\n";
      return 0;
    }

    std::map<std::string, drc::transform_t>::const_iterator it;
    it ==cartpos_out.find(link_name);
    if(it!=cartpos_out.end())// cart pos exists
    {
     drc::transform_t link_tf = cartpos_out.find( link_name)->second;
     TransformLCMToKDLFrame(link_tf,T_body_link);
        /*drc::transform_t link_tf_old =cartpos_out.find( link_name)->second;
        link_tf.utime =cartpos_out_utime;
        link_tf.trans[0]  = link_tf_old.translation.x;
        link_tf.trans[1]  = link_tf_old.translation.y;
        link_tf.trans[2]  = link_tf_old.translation.z;
        link_tf.quat[0]  = link_tf_old.rotation.w;
        link_tf.quat[1]  = link_tf_old.rotation.x;
        link_tf.quat[2]  = link_tf_old.rotation.y;
        link_tf.quat[3]  = link_tf_old.rotation.z;*/
      return true;
    }
    else
    {
      std::cerr << "ERROR: TfSolver could not find tf for " << link_name << std::endl;
      return false;
    }

  };


} //end namespace


