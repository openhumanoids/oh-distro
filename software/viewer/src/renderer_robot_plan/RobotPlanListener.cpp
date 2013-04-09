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
    _viewer(viewer),
    _robot_name("atlas"),
    _in_motion_keyframe_index(-1)
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
				       &renderer_robot_plan::RobotPlanListener::handleRobotUrdfMsg,
				       this);  
    _urdf_subscription_on =  true;
    lcm->subscribe("CANDIDATE_ROBOT_PLAN", &renderer_robot_plan::RobotPlanListener::handleRobotPlanMsg, this); //&this ?
    lcm->subscribe("CANDIDATE_MANIP_PLAN", &renderer_robot_plan::RobotPlanListener::handleManipPlanMsg, this); 
    
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
    
    _is_manip_plan =false;
    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
  }
  
  RobotPlanListener::~RobotPlanListener() {
   // _collision_detector->clear_collision_objects();
  }


  //=============message callbacks

void RobotPlanListener::handleRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_plan_t* msg)						 
  {
    cout << "\n received robot plan message\n";

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
    _is_manip_plan =false;
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
    _gl_robot_list.clear();

    int count=msg->num_states-1; 	   	// always display the last state in the plan
    for (uint i = 0; i <(uint)num_states; i++)
    {
      drc::robot_state_t state_msg  = msg->plan[count];
    	std::stringstream oss;
    	oss << _robot_name << "_"<< count; 
    	shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,true,oss.str()));
			_gl_robot_list.insert(_gl_robot_list.begin(),new_object_ptr);
			_gl_robot_list[0]->set_state(state_msg);
			count-=inc;
    }//end for num of states in robot_plan msg;
   	
		_last_plan_msg_timestamp = bot_timestamp_now(); //initialize
    bot_viewer_request_redraw(_viewer);
  } // end handleMessage
  
  
  void RobotPlanListener::handleManipPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_plan_w_keyframes_t* msg)						 
  {
    cout << "\n received robot manipulation plan message\n";

    if (!_urdf_parsed)
    {
      cout << "\n handleManipPlanMsg: Waiting for urdf to be parsed" << endl;
      return;
    }
    if(_urdf_subscription_on)
    {
      cout << "\n handleManipPlanMsg: unsubscribing from _urdf_subscription" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }
    
    _is_manip_plan =true;

    // 0. Make Local copy as drc::robot_plan_t to later output
    drc::robot_plan_w_keyframes_t msgcopy =*msg;
    revieved_plan_.utime = msgcopy.utime;
    revieved_plan_.robot_name = msgcopy.robot_name;
    revieved_plan_.num_states = msgcopy.num_states;
    revieved_plan_.plan = msgcopy.plan;
    revieved_plan_.num_bytes = msgcopy.num_bytes;
    revieved_plan_.matlab_data = msgcopy.matlab_data;
    
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
    _gl_robot_list.clear();
    _gl_robot_keyframe_list.clear();
    _keyframe_timestamps.clear();
    
    int count=0; 	   	// always display the last state in the plan
    for (uint i = 0; i <(uint)msg->num_states; i++)
    {

       if(msg->is_keyframe[i]) 
       {
        drc::robot_state_t state_msg  = msg->plan[i];
	      std::stringstream oss;
	      oss << _robot_name << "_" << "keyframe"<< "_"<< count; 
	      shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,true,oss.str()));
       _gl_robot_keyframe_list.push_back(new_object_ptr);
       _gl_robot_keyframe_list[count]->enable_whole_body_selection(true);
       _gl_robot_keyframe_list[count]->set_state(state_msg);
       _keyframe_timestamps.push_back(state_msg.utime);
        count++;
       }
    }//end for num of states in robot_plan msg;
    
    
    count=msg->num_states-1; 	   	// always display the last state in the plan
    for (uint i = 0; i <(uint)num_states; i++)
    {
      drc::robot_state_t state_msg  = msg->plan[count];
    	std::stringstream oss;
    	oss << _robot_name << "_"<< count; 
    	shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,false,oss.str()));
			_gl_robot_list.insert(_gl_robot_list.begin(),new_object_ptr);
			_gl_robot_list[0]->set_state(state_msg);
			count-=inc;
    }//end for num of states in robot_plan msg;

    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
    bot_viewer_request_redraw(_viewer);  
  
  }

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
  
  
  void RobotPlanListener::commit_robot_plan(int64_t utime,std::string &channel)
  {
    drc::robot_plan_t msg = revieved_plan_;
    msg.utime = utime;
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
  

} //namespace renderer_robot_plan

//      double tic = bot_timestamp_now();
//      double toc = bot_timestamp_now();
//      cout<< "elaspsed "<<(toc-tic)/1000000 << endl;
