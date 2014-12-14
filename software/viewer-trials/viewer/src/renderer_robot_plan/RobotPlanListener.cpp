#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/desired_grasp_state_t.hpp"
#include "lcmtypes/drc/driving_affordance_status_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"

#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
namespace renderer_robot_plan 
{
  //==================constructor / destructor
  
RobotPlanListener::RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer, int operation_mode):
    _urdf_parsed(false), _lcm(lcm), _viewer(viewer), _robot_name("atlas"), _in_motion_keyframe_index(-1), _is_manip_plan(false), _is_manip_map(false), 
    _active_breakpoint(0), _num_breakpoints(0), _retractable_cycle_counter(0), _current_plan_committed(false){

  if(!lcm->good()){
    cerr << "\nLCM Not Good: Robot State Handler" << endl;
    return;
  }

  // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
  _urdf_subscription = _lcm->subscribe("ROBOT_MODEL", &renderer_robot_plan::RobotPlanListener::handleRobotUrdfMsg, this);  
  _urdf_subscription_on =  true;

  lcm->subscribe("CANDIDATE_MANIP_PLAN", &renderer_robot_plan::RobotPlanListener::handleManipPlanMsg, this);
  lcm->subscribe("EST_ROBOT_STATE", &renderer_robot_plan::RobotPlanListener::handleRobotStateMsg, this); 

  _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
}
  
RobotPlanListener::~RobotPlanListener() {
}

void RobotPlanListener::handleManipPlanMsg(const lcm::ReceiveBuffer* rbuf, const string& chan, const drc::robot_plan_w_keyframes_t* msg){

  //cout << "\n received robot manipulation plan message\n";
  if (!_urdf_parsed){
    cout << "\n handleManipPlanMsg: Waiting for urdf to be parsed" << endl;
    return;
  }

  if(_urdf_subscription_on){
    cout << "\n handleManipPlanMsg: unsubscribing from _urdf_subscription" << endl;
    _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
    _urdf_subscription_on =  false; 	
  }
    

  // 0. Make Local copy as drc::robot_plan_t to later output
  drc::robot_plan_w_keyframes_t msgcopy =*msg;
  _received_plan.utime = msgcopy.utime;
  _received_plan.robot_name = msgcopy.robot_name;
  _received_plan.num_states = msgcopy.num_states;
  _received_plan.plan = msgcopy.plan;
  _received_plan.plan_info = msgcopy.plan_info;
  _received_plan.num_bytes = msgcopy.num_bytes;
  _received_plan.matlab_data = msgcopy.matlab_data;
  _received_plan.num_grasp_transitions = msgcopy.num_grasp_transitions;
  if(msgcopy.num_grasp_transitions>0)
    _received_plan.grasps = msgcopy.grasps;

  _num_breakpoints = msg->num_breakpoints;

     
  int max_num_states = 100;
  int num_states = 0;
  int inc = 1;
  if (msg->num_states > max_num_states) {
    inc = ceil(msg->num_states/max_num_states);	
    inc = min(max(inc,1),max_num_states);	
    num_states = max_num_states;   
  }else 
    num_states = msg->num_states;   

    //clear stored data
    _gl_robot_list.clear();
    _gl_robot_keyframe_list.clear();
    _keyframe_timestamps.clear();
    _breakpoint_indices.clear();
    
    for (uint i = 0; i <(uint)msg->num_states; i++)
    {
        if(msg->is_breakpoint[i]) 
        {
            _breakpoint_indices.push_back(i);
        }
    }
    bool append_currenthandstate = (_breakpoint_indices.size()==0);
    //cout << "append_currenthandstate: "<< append_currenthandstate << " " << _breakpoint_indices.size() << endl;
    int count=0; 	   	// always display the last state in the plan
    for (uint i = 0; i <(uint)msg->num_states; i++)
    {
        if(msg->is_keyframe[i]) 
        {
            drc::robot_state_t state_msg  = msg->plan[i];
            // Merge in grasp state transitions into the plan states.
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
    //for (uint i = 0; i <(uint)num_states; i++)
    while (count >= 0)
    {
        drc::robot_state_t state_msg  = msg->plan[count];
        // Merge in grasp state transitions into the plan states.
        std::stringstream oss;
        oss << _robot_name << "_"<< count; 
        shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,false,oss.str()));
        _gl_robot_list.insert(_gl_robot_list.begin(),new_object_ptr);
        _gl_robot_list[0]->set_state(state_msg);
        count-=inc;
    }//end for num of states in robot_plan msg; 
    
    
  if(count!=0){ // always include the initial state too/.
    count = 0;
    drc::robot_state_t state_msg  = msg->plan[count];
    // Merge in grasp state transitions into the plan states.
    std::stringstream oss;
    oss << _robot_name << "_"<< count; 
    shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,false,oss.str()));
    _gl_robot_list.insert(_gl_robot_list.begin(),new_object_ptr);
    _gl_robot_list[0]->set_state(state_msg); 
  }

  _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
  bot_viewer_request_redraw(_viewer);  
  
}


void RobotPlanListener::handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, const  drc::robot_urdf_t* msg){

  if(_urdf_parsed ==false){
    // Received robot urdf string. Store it internally and get all available joints.
    cout<< "\nurdf handler @ RobotPlanListener" << endl;
    _robot_name      = msg->robot_name;
    _urdf_xml_string = msg->urdf_xml_string;
    cout<< "\nReceived urdf_xml_string of robot [" << msg->robot_name << "], storing it internally as a param" << endl;
	
    _base_gl_robot = shared_ptr<GlKinematicBody>(new GlKinematicBody(_urdf_xml_string));
    _base_gl_robot->disable_joint_limit_enforcement();
    cout<< "Number of Joints: " << _base_gl_robot->get_num_joints() <<endl;
    
    //remember that we've parsed the urdf already
    _urdf_parsed = true;
        
  }//  if(_urdf_parsed ==false) 
} 


void RobotPlanListener::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const string& chan, const drc::robot_state_t* msg){ 
  if (!_urdf_parsed){
    return;
  }

  if(_urdf_subscription_on){			
    cout << "\n handleRobotStateMsg: unsubscribing from _urdf_subscription" << endl;
    _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
    _urdf_subscription_on =  false; 	
  }

  _last_robotstate_msg = (*msg);
} 


}
