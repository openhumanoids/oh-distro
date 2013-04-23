#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "RobotStateListener.hpp"

using namespace std;
using namespace boost;

namespace renderer_affordances 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  /*RobotStateListener::RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer):
    _lcm(lcm),
    _viewer(viewer)*/
  RobotStateListener::RobotStateListener(RendererAffordances* parent_renderer):
   _parent_renderer(parent_renderer),_robot_state_received(false)
  {

    _lcm = _parent_renderer->lcm; 
    //lcm ok?
    if(!_lcm->good())
    {
      cerr << "\nLCM Not Good: Robot State Handler" << endl;
      return;
    }
    T_body_world = KDL::Frame::Identity();

    _parent_renderer->last_state_msg_timestamp = 0;
     _parent_renderer->robot_name = "atlas";//default
   //(*_parent_renderer->robot_name_ptr) = "atlas"; 

    // Subscribe to Robot_state. 
    _lcm->subscribe("EST_ROBOT_STATE", &RobotStateListener::handleRobotStateMsg, this); 
  }
  
  RobotStateListener::~RobotStateListener() {}


  //=============message callbacks

void RobotStateListener::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_state_t* msg)						 
  { 
      if(!_robot_state_received)
      _robot_state_received = true;
  
  	  KDL::Frame  T_world_body;
  	    
      T_world_body.p[0]= msg->origin_position.translation.x;
	    T_world_body.p[1]= msg->origin_position.translation.y;
	    T_world_body.p[2]= msg->origin_position.translation.z;		    
	    T_world_body.M =  KDL::Rotation::Quaternion(msg->origin_position.rotation.x, msg->origin_position.rotation.y, msg->origin_position.rotation.z, msg->origin_position.rotation.w);

      T_body_world=T_world_body.Inverse();   
      
   
      last_robotstate_msg = (*msg);
      _parent_renderer->last_state_msg_timestamp = msg->utime;
       _parent_renderer->robot_name = msg->robot_name;
      //(*_parent_renderer->robot_name_ptr)  = msg->robot_name;
      
       
    
  } // end handleMessage


} //end namespace


