#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
namespace renderer_robot_plan 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  RobotPlanListener::RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer):
    _urdf_parsed(false),
    _lcm(lcm),
    _viewer(viewer)

  {
     _collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    //lcm ok?
    if(!lcm->good())
      {
	cerr << "\nLCM Not Good: Robot State Handler" << endl;
	return;
      }

    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = _lcm->subscribe("ROBOT_MODEL", 
				       &renderer_robot_plan::RobotPlanListener::handleRobotUrdfMsg,
				       this);  
    _urdf_subscription_on =  true;
    lcm->subscribe("CANDIDATE_ROBOT_PLAN", &renderer_robot_plan::RobotPlanListener::handleRobotPlanMsg, this); //&this ?
    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
  }
  
  RobotPlanListener::~RobotPlanListener() {
    _collision_detector->clear_collision_objects();
  }


  //=============message callbacks

void RobotPlanListener::handleRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_plan_t* msg)						 
  {
    if (!_urdf_parsed)
      {
	cout << "\n handleRobotPlanMsg: Waiting for urdf to be parsed" << endl;
	return;
      }
   if(_urdf_subscription_on)
     {
       cout << "\n handleRobotPlanMsg: unsubscribing from _urdf_subscription" << endl;
       _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
	 _urdf_subscription_on =  false; 	
    }
   
       // 0. Make Local copy to later output
        revieved_plan_ = *msg;
   
  	int max_num_states = 20;
  	int num_states = 0;
   	int inc = 1;
 	if (msg->num_states > max_num_states) {
		inc = ceil(msg->num_states/max_num_states);	
		inc = min(max(inc,1),max_num_states);	
		num_states = max_num_states;   
	}   
	else 
		num_states = msg->num_states;   

    //clear stored data
    int old_list_size = _gl_robot_list.size();
    if(old_list_size!=num_states){
      _gl_robot_list.clear();
      _collision_detector.reset();
      _collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    }
    int count=0;
    for (uint i = 0; i <(uint)num_states; i++)
    {
      drc::robot_state_t state_msg  = msg->plan[count];

     //shared_ptr<visualization_utils::GlKinematicBody> new_object_ptr(new visualization_utils::GlKinematicBody(*_base_gl_robot));
     if(old_list_size!=num_states){
      std::stringstream oss;
      oss << _robot_name << "_"<< count; 
      shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,_collision_detector,true,oss.str()));
      _gl_robot_list.push_back(new_object_ptr);
      }
      _gl_robot_list[i]->set_state(state_msg);

	  count+=inc;
    }//end for num of states in robot_plan msg;
    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
    bot_viewer_request_redraw(_viewer);
  } // end handleMessage


 void RobotPlanListener::handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::robot_urdf_t* msg) 
  {

    if(_urdf_parsed ==false) 
    {
     // Received robot urdf string. Store it internally and get all available joints.
      cout<< "\nurdf handler @ RobotPlanListener" << endl;
    _robot_name      = msg->robot_name;
    _urdf_xml_string = msg->urdf_xml_string;
    cout<< "\nReceived urdf_xml_string of robot [" 
	<< msg->robot_name << "], storing it internally as a param" << endl;
	      
    _base_gl_robot = shared_ptr<GlKinematicBody>(new GlKinematicBody(_urdf_xml_string));
    cout<< "Number of Joints: " << _base_gl_robot->get_num_joints() <<endl;

    //remember that we've parsed the urdf already
    _urdf_parsed = true;
        
    }//  if(_urdf_parsed ==false) 

  } 

} //namespace renderer_robot_plan


