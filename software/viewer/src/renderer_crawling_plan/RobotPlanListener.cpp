#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
namespace renderer_crawling_plan 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  RobotPlanListener::RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer, int operation_mode):
    _urdf_parsed(false),
    _lcm(lcm),
    _viewer(viewer),
    _robot_name("atlas"),
    _in_motion_keyframe_index(-1),
    _is_keyframe_plan(false),
		_aprvd_walking_goal_in_cache(false),
		_is_multi_approve_plan(false),
		_active_breakpoint(0),
		_num_breakpoints(0),
		_plan_paused(false)
  {
     //_collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    //lcm ok?
    if(!lcm->good())
      {
	      cerr << "\nLCM Not Good: Robot State Handler" << endl;
	      return;
      }

    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = _lcm->subscribe("ROBOT_MODEL", 
				       &renderer_crawling_plan::RobotPlanListener::handleRobotUrdfMsg,
				       this);  
    _urdf_subscription_on =  true;
    lcm->subscribe("CANDIDATE_CRAWLING_PLAN", &renderer_crawling_plan::RobotPlanListener::handleRobotPlanMsg, this); //&this
    lcm->subscribe("CRAWLING_NAV_GOAL", &renderer_crawling_plan::RobotPlanListener::handleAprvWalkingGoalMsg, this);  
    lcm->subscribe("CONTROLLER_STATUS", &renderer_crawling_plan::RobotPlanListener::handleControllerStatusMsg, this);  


    // Pre-load hand URDFS
    std::string _left_hand_urdf_xml_string,_right_hand_urdf_xml_string;
    if(!load_hand_urdfs(_left_hand_urdf_xml_string,_right_hand_urdf_xml_string))
       cerr << "\nHand Urdfs Not Found" << endl;
    else{
        string unique_hand_name = "lhand_local_copy";
       _gl_left_hand = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody (_left_hand_urdf_xml_string,true,unique_hand_name));
       unique_hand_name = "rhand_local_copy"; 
       _gl_right_hand = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody (_right_hand_urdf_xml_string,true,unique_hand_name));
    }
    
     std::string _left_foot_urdf_xml_string,_right_foot_urdf_xml_string;
    if(!load_foot_urdfs(_left_foot_urdf_xml_string,_right_foot_urdf_xml_string))
       cerr << "\n Foot Urdfs Not Found" << endl;
    else{
        string unique_foot_name = "lfoot_local_copy";
       _gl_left_foot = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody (_left_foot_urdf_xml_string,true,unique_foot_name));
       unique_foot_name = "rfoot_local_copy"; 
       _gl_right_foot = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody (_right_foot_urdf_xml_string,true,unique_foot_name));
    }

    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
  }
  
  RobotPlanListener::~RobotPlanListener() {
   // _collision_detector->clear_collision_objects();
  }


  //=============message callbacks




void RobotPlanListener::handleRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
	const string& chan, const drc::robot_plan_t* msg)
  {
    //cout << "\n received robot plan message\n";
    _plan_paused = false;
    if (!_urdf_parsed){
      cout << "\n handleRobotPlanMsg: Waiting for urdf to be parsed" << endl;
      return;
    }
    if(_urdf_subscription_on){
      cout << "\n handleRobotPlanMsg: unsubscribing from _urdf_subscription" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }
    activate_crawling_plan();

    // 0. Make Local copy to later output
    _received_plan = *msg;

    int num_states = 0;
    int inc = 1;
    
    /*
    int max_num_states = 20;
    if (msg->num_states > max_num_states) {
      inc = ceil(msg->num_states/max_num_states);	
      inc = min(max(inc,1),max_num_states);	
      num_states = max_num_states;   
    }else{
      num_states = msg->num_states;   
      }
    */
      num_states = msg->num_states;   

    //clear stored data
    _gl_robot_list.clear();

    //cout << "a\n";
    int count=msg->num_states-1; // always display the last state in the plan
    for (uint i = 0; i <(uint)num_states; i++){
      drc::robot_state_t state_msg  = msg->plan[count];
      std::stringstream oss;
      oss << _robot_name << "_"<< count; 
      shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,true,oss.str()));
      _gl_robot_list.insert(_gl_robot_list.begin(),new_object_ptr);
      _gl_robot_list[0]->set_state(state_msg);
      count-=inc;
    }//end for num of states in robot_plan msg;
    //	cout << "b\n";

    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
    bot_viewer_request_redraw(_viewer);
  } // end handleMessage
  

  
  //-------------------------------------------------------------------

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
	
	  //bot_gtk_gl_drawing_area_set_context(this->_viewer->gl_area); // Prevents conflict with cam renderer which messes with the gl context    
    _base_gl_robot = shared_ptr<GlKinematicBody>(new GlKinematicBody(_urdf_xml_string));
    cout<< "Number of Joints: " << _base_gl_robot->get_num_joints() <<endl;

    //remember that we've parsed the urdf already
    _urdf_parsed = true;
        
    }//  if(_urdf_parsed ==false) 

  } 

  //-------------------------------------------------------------------

  void RobotPlanListener::handleAprvWalkingGoalMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::walking_goal_t* msg)						 
  {
		_received_walking_goal = *msg;
    _aprvd_walking_goal_in_cache = true;
     purge_current_plan();
  }
 
  void RobotPlanListener::handleControllerStatusMsg(const lcm::ReceiveBuffer* rbuf,
                                                 const string& chan, 
                                                 const drc::controller_status_t* msg)                                                
  {
     _controller_utime = msg->controller_utime;
     _controller_status = msg->state;
  }  
  
    //-------------------------------------------------------------------

  void RobotPlanListener::commit_walking_goal(int64_t utime,std::string &channel)
  {
    drc::walking_goal_t msg = _received_walking_goal;
    msg.utime = utime;
    _lcm->publish(channel, &msg);
    _aprvd_walking_goal_in_cache = false; //clear flag on commit
  }


  void RobotPlanListener::commit_plan_control(int64_t utime, std::string &channel,bool pause, bool terminate)
  {
  
    drc::plan_control_t  msg;
    msg.utime = utime;
    if((pause)&&(!terminate)){
      msg.control = msg.PAUSE;
      _plan_paused = true;
    }
    else if((!pause)&&(!terminate)){
      msg.control = msg.UNPAUSE;
      _plan_paused = false;
    }
    else{
       msg.control = msg.TERMINATE; 
       _plan_paused = false;
    }
    _lcm->publish(channel, &msg);
  }  
  
  bool RobotPlanListener::load_hand_urdfs(std::string &_left_hand_urdf_xml_string,std::string &_right_hand_urdf_xml_string)
  {
  
    string urdf_models_path = string(getModelsPath()) + "/mit_gazebo_models/mit_robot_hands/"; 
    

    vector<string> urdf_files = vector<string>();
    get_URDF_filenames_from_dir(urdf_models_path.c_str(),urdf_files);

    std::string filename, ext;

        
    filename ="sandia_hand_left";
    ext=".urdf";
    
    std::vector<std::string>::const_iterator found;
    found = std::find(urdf_files.begin(),urdf_files.end(), filename);
    if (found != urdf_files.end()) {
      std::stringstream oss;   
      oss << urdf_models_path  << filename << ext;   
      get_xmlstring_from_file(oss.str(),_left_hand_urdf_xml_string);     
    }
    else{
      return false;
    }
    
    filename ="sandia_hand_right";
    ext=".urdf";
    
    if (found != urdf_files.end()) {
      std::stringstream oss;   
      oss << urdf_models_path  << filename << ext;   
      get_xmlstring_from_file(oss.str(),_right_hand_urdf_xml_string);
    }
    else{
      return false;
    }
  
     return true;
  }

  bool RobotPlanListener::load_foot_urdfs(std::string &_left_foot_urdf_xml_string,std::string &_right_foot_urdf_xml_string)
  {
  
    string urdf_models_path = string(getModelsPath()) + "/mit_gazebo_models/mit_robot_feet/"; 
    

    vector<string> urdf_files = vector<string>();
    get_URDF_filenames_from_dir(urdf_models_path.c_str(),urdf_files);

    std::string filename, ext;

        
    filename ="l_foot";
    ext=".urdf";
    
    std::vector<std::string>::const_iterator found;
    found = std::find(urdf_files.begin(),urdf_files.end(), filename);
    if (found != urdf_files.end()) {
      std::stringstream oss;   
      oss << urdf_models_path  << filename << ext;   
      get_xmlstring_from_file(oss.str(),_left_foot_urdf_xml_string);     
    }
    else{
      return false;
    }
    
    filename ="r_foot";
    ext=".urdf";
    
    if (found != urdf_files.end()) {
      std::stringstream oss;   
      oss << urdf_models_path  << filename << ext;   
      get_xmlstring_from_file(oss.str(),_right_foot_urdf_xml_string);
    }
    else{
      return false;
    }
  
     return true;
  }  

} //namespace renderer_crawling_plan

