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
  RobotPlanListener::RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer, int operation_mode):
    _urdf_parsed(false),
    _lcm(lcm),
    _viewer(viewer),
    _robot_name("atlas"),
    _in_motion_keyframe_index(-1),
    _is_manip_plan(false),
    _is_manip_map(false),
		_aprvd_footstep_plan_in_cache(false),
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
				       &renderer_robot_plan::RobotPlanListener::handleRobotUrdfMsg,
				       this);  
    _urdf_subscription_on =  true;
    lcm->subscribe("CANDIDATE_ROBOT_PLAN", &renderer_robot_plan::RobotPlanListener::handleRobotPlanMsg, this); //&this ?
    if ( operation_mode ==0 ){ // typical
      lcm->subscribe("CANDIDATE_MANIP_PLAN", &renderer_robot_plan::RobotPlanListener::handleManipPlanMsg, this);
    }else if(operation_mode ==1){ // sent to base shaper:
      lcm->subscribe("COMMITTED_ROBOT_PLAN", &renderer_robot_plan::RobotPlanListener::handleRobotPlanMsg, this);
    }else if(operation_mode ==2){ // what would be published by robot shaper:
      lcm->subscribe("COMMITTED_ROBOT_PLAN_COMPRESSED_LOOPBACK", &renderer_robot_plan::RobotPlanListener::handleRobotPlanMsg, this);
    }
    lcm->subscribe("CANDIDATE_MANIP_MAP", &renderer_robot_plan::RobotPlanListener::handleAffIndexedRobotPlanMsg, this);  
    
    lcm->subscribe("CANDIDATE_FOOTSTEP_PLAN", &renderer_robot_plan::RobotPlanListener::handleCanFootStepPlanMsg, this);  
    lcm->subscribe("APPROVED_FOOTSTEP_PLAN", &renderer_robot_plan::RobotPlanListener::handleAprvFootStepPlanMsg, this);  

    lcm->subscribe("CONTROLLER_STATUS", &renderer_robot_plan::RobotPlanListener::handleControllerStatusMsg, this);  


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
    activate_walking_plan();

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
  void RobotPlanListener::handleManipPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_plan_w_keyframes_t* msg)						 
  {
    //cout << "\n received robot manipulation plan message\n";
    _plan_paused = false;
    if (!_urdf_parsed){
      cout << "\n handleManipPlanMsg: Waiting for urdf to be parsed" << endl;
      return;
    }
    if(_urdf_subscription_on)
    {
      cout << "\n handleManipPlanMsg: unsubscribing from _urdf_subscription" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }
    
    activate_manip_plan();

    // 0. Make Local copy as drc::robot_plan_t to later output
    drc::robot_plan_w_keyframes_t msgcopy =*msg;
    _received_plan.utime = msgcopy.utime;
    _received_plan.robot_name = msgcopy.robot_name;
    _received_plan.num_states = msgcopy.num_states;
    _received_plan.plan = msgcopy.plan;
    _received_plan.num_bytes = msgcopy.num_bytes;
    _received_plan.matlab_data = msgcopy.matlab_data;
    _received_plan.num_grasp_transitions = msgcopy.num_grasp_transitions;
    if(msgcopy.num_grasp_transitions>0)
     _received_plan.grasps = msgcopy.grasps;
     _num_breakpoints = msg->num_breakpoints;
     if(msg->num_breakpoints>0) 
     {
       // It is a multi-approval plan
       activate_multi_approval();  
     } 
     else
      deactivate_multi_approval();
     
    int max_num_states = 30;
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
    _breakpoint_indices.clear();

    int count=0; 	   	// always display the last state in the plan
    for (uint i = 0; i <(uint)msg->num_states; i++)
        {
            if(msg->is_keyframe[i]) 
                {
                    drc::robot_state_t state_msg  = msg->plan[i];
                    // Merge in grasp state transitions into the plan states.
                    appendHandStatesToStateMsg(msg,&state_msg);
                    std::stringstream oss;
                    oss << _robot_name << "_" << "keyframe"<< "_"<< count; 
                    shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,true,oss.str()));
                    _gl_robot_keyframe_list.push_back(new_object_ptr);
                    _gl_robot_keyframe_list[count]->enable_whole_body_selection(true);
                    _gl_robot_keyframe_list[count]->set_state(state_msg);
                    _keyframe_timestamps.push_back(state_msg.utime);
                    count++;
                }
       
            if(msg->is_breakpoint[i]) 
                {
                    _breakpoint_indices.push_back(i);
                }
        }//end for num of states in robot_plan msg;

    count=msg->num_states-1; 	   	// always display the last state in the plan
    for (uint i = 0; i <(uint)num_states; i++)
        {
            drc::robot_state_t state_msg  = msg->plan[count];
            // Merge in grasp state transitions into the plan states.
            appendHandStatesToStateMsg(msg,&state_msg);
            std::stringstream oss;
            oss << _robot_name << "_"<< count; 
            shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,false,oss.str()));
            _gl_robot_list.insert(_gl_robot_list.begin(),new_object_ptr);
            _gl_robot_list[0]->set_state(state_msg);
            count-=inc;
        }//end for num of states in robot_plan msg; 
    
    
    if(count!=0) // always include the initial state too/.
    {
      count = 0;
      drc::robot_state_t state_msg  = msg->plan[count];
      // Merge in grasp state transitions into the plan states.
      appendHandStatesToStateMsg(msg,&state_msg);
    	std::stringstream oss;
    	oss << _robot_name << "_"<< count; 
    	shared_ptr<InteractableGlKinematicBody> new_object_ptr(new InteractableGlKinematicBody(*_base_gl_robot,false,oss.str()));
			_gl_robot_list.insert(_gl_robot_list.begin(),new_object_ptr);
			_gl_robot_list[0]->set_state(state_msg); 
    }

    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
    bot_viewer_request_redraw(_viewer);  
  
  }
    //-------------------------------------------------------------------
  
  void RobotPlanListener::handleAffIndexedRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::aff_indexed_robot_plan_t* msg)						 
  {
    cout << "\n received aff_indexed_robot_plan for pre-execution_approval message\n";
    _plan_paused = false;
    if (!_urdf_parsed)
    {
      cout << "\n handleAffIndexedRobotPlanMsg: Waiting for urdf to be parsed" << endl;
      return;
    }
    if(_urdf_subscription_on)
    {
      cout << "\n handleRobotPlanMsg: unsubscribing from _urdf_subscription" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }
    
    activate_manip_map();
    
   // 0. Make Local copy to later output
    _received_map = *msg;

  	int max_num_states = 30;
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

  void RobotPlanListener::handleAprvFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::footstep_plan_t* msg)						 
  {
		_received_footstep_plan = *msg;
    _aprvd_footstep_plan_in_cache = true;
     purge_current_plan();
  }
  
   void RobotPlanListener::handleCanFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::footstep_plan_t* msg)						 
  {
    if(_aprvd_footstep_plan_in_cache)
      _aprvd_footstep_plan_in_cache = false;
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

  void RobotPlanListener::commit_footstep_plan(int64_t utime,std::string &channel)
  {
    drc::footstep_plan_t msg = _received_footstep_plan;
    msg.utime = utime;
    _lcm->publish(channel, &msg);
    _aprvd_footstep_plan_in_cache = false; //clear flag on commit
  }


  void RobotPlanListener::commit_robot_plan(int64_t utime,std::string &channel)
  {
    drc::robot_plan_t msg = _received_plan;
    msg.utime = utime;
    msg.arms_control_type = msg.POS_ONLY;
    msg.legs_control_type = msg.POS_ONLY;
    _lcm->publish(channel, &msg);
  }
  
  void RobotPlanListener::commit_manip_plan(int64_t utime,std::string &channel)
  {
    drc::robot_plan_t msg;
    commit_compliant_manip_plan(utime,channel,msg.POS_ONLY,msg.POS_ONLY);
  }
  
  void RobotPlanListener::commit_compliant_manip_plan(int64_t utime,std::string &channel,int arms_control_type,int legs_control_type)
  {
  //cout << "in commit compliant manip plan\n";
   if(!is_multi_approval_plan()) {
      drc::robot_plan_t msg = _received_plan;
      msg.utime = utime;
      msg.arms_control_type = arms_control_type;
      msg.legs_control_type = legs_control_type;
      for(uint i=0;i<msg.num_states;i++)
        cout << msg.plan[i].utime << endl;
      _lcm->publish(channel, &msg);
      //cout << msg.num_states <<" not a multi approval plan\n";
    }
    else {
     //cout << "is a multi approval plan\n";
      drc::robot_plan_t msg;

      msg.utime = utime;
      msg.robot_name = _received_plan.robot_name;
      msg.num_bytes = _received_plan.num_bytes;
      msg.matlab_data = _received_plan.matlab_data;
      msg.num_grasp_transitions = 0;  
      msg.arms_control_type = arms_control_type;
      msg.legs_control_type = legs_control_type;    
      int num_states=0;
 
      // example breakpoints [0 9 10 19]
      // grasp transitions   [0(+) 10(-)]
      // _active_breakpoint=0; 0:9 [G+,pause,M+]
      // _active_breakpoint=1; 9:10 [G-]   
      // _active_breakpoint=2; 10:19 [M-]
      // _active_breakpoint=3; [19 0] [G-,G+]
      
      
      
      int start_ind = _breakpoint_indices[_active_breakpoint];
      int64_t start_utime = _received_plan.plan[start_ind].utime;  
      
     // check to see if _received_plan.plan[start_ind].utime is a grasp transition
     // if yes commit a grasp and usleep(1000000) then commit plan.  
      if(_received_plan.num_grasp_transitions>0)
      {
          size_t k_max = 0;
          bool is_grasp_transition=false;
          bool bi_handed=false;
          for(size_t k=0;k<_received_plan.num_grasp_transitions;k++)
          {
          
            if(_received_plan.plan[start_ind+1].utime >= _received_plan.grasps[k].utime){
             k_max = std::max(k_max,k);
             is_grasp_transition=true;
            }            
          }
          // get the latest grasp transition index.
          if(is_grasp_transition){            
            //cout << "commit grasp at:" << start_ind << endl;
            std::vector<int> kmax_inds;
            //_received_plan.grasps[k].utime can be non-unique upto 2 (getting both inds)
             for(size_t k=0;k<_received_plan.num_grasp_transitions;k++)
            {
              if(_received_plan.grasps[k].utime==_received_plan.grasps[k_max].utime)
                  kmax_inds.push_back(k);
            }
            
            string channel ="COMMITTED_GRASP";
            commit_desired_grasp_state(utime,channel,kmax_inds);
            msg.num_grasp_transitions = 1;
            msg.grasps.push_back(_received_plan.grasps[k_max]); 
            usleep(1000000);
         }
      }

          
      if(_active_breakpoint<_breakpoint_indices.size()-1) 
      {
      // Get plan from _breakpoint_indices[_active_breakpoint]: _breakpoint_indices[_active_breakpoint+1]
        for (uint i = _breakpoint_indices[_active_breakpoint]; i <=(uint)_breakpoint_indices[_active_breakpoint+1]; i++)
        {
          
          msg.plan.push_back(_received_plan.plan[i]);
          msg.plan[num_states].utime =msg.plan[num_states].utime - start_utime;
          num_states++;
          //cout <<"i :" << i << endl;          
        }   // end for     
        //cout <<"_active_breakpoint :" <<_active_breakpoint << "   _breakpoint_indices[_active_breakpoint] :" << _breakpoint_indices[_active_breakpoint]<< endl;
        _active_breakpoint++;
      }
      else
      {
        //cout <<"_active_breakpoint :" <<_active_breakpoint << "   _breakpoint_indices[_active_breakpoint] :" << _breakpoint_indices[_active_breakpoint]<< endl;
         //close the loop
        //get states until the end, and append states from beginning to the first breakpoint
        //[{_breakpoint_indices[_active_breakpoint]:_breakpoint_indices.size()-1}, {0:_breakpoint_indices[0]}] 

        int counter = 0;
        for (uint i = _breakpoint_indices[_active_breakpoint]; i <= (uint)_breakpoint_indices[_breakpoint_indices.size()-1]; i++)
        {

          msg.plan.push_back(_received_plan.plan[i]);
          msg.plan[num_states].utime =msg.plan[num_states].utime - start_utime;
          num_states++;
          //cout <<"i :" << i << endl;
        } // end for
        int64_t cycle_delay_offset = msg.plan[num_states].utime + _received_plan.plan[1].utime - _received_plan.plan[0].utime;
        for (uint i = 0; i <=(uint)_breakpoint_indices[0]; i++)
        {
          msg.plan.push_back(_received_plan.plan[i]);
          msg.plan[num_states].utime =msg.plan[num_states].utime + cycle_delay_offset;
          num_states++;
          //cout <<"i :" << i << endl;
        } // end for        
        _active_breakpoint = 0; // completes the cycle.
      }// end if else
      msg.num_states = num_states;
      
      /*for(uint i=0;i<msg.num_states;i++)
        cout << msg.plan[i].utime << endl;  */  
      _lcm->publish(channel, &msg);
    }// end else;
  }
  
  void RobotPlanListener::commit_manip_map(int64_t utime, std::string &channel)
  {
  
    drc::aff_indexed_robot_plan_t msg = _received_map;
    msg.utime = utime;
    _lcm->publish(channel, &msg);
    
    //send out a manip map status message - for the driving renderer at the base end to use 
    if(msg.num_states > 0){
        int no_ees = msg.aff_index[0].num_ees; 
        for (int j=0; j < no_ees; j++){ 
            drc::driving_affordance_status_t msg_s;
            msg_s.utime = utime; 
            msg_s.aff_type = msg.aff_index[0].aff_type;
            msg_s.aff_uid = msg.aff_index[0].aff_uid;
            msg_s.dof_name = msg.aff_index[0].dof_name[j]; 
            msg_s.ee_name = msg.aff_index[0].ee_name[j]; 

            double max = -100000;
            double min =  100000;
            for(int i=0; i < msg.num_states; i++){
                max = fmax(max,msg.aff_index[i].dof_value[j]); 
                min = fmin(min,msg.aff_index[i].dof_value[j]); 
            }

            fprintf(stderr, "DOF : %s -> Max : %f Min : %f\n",  msg_s.dof_name.c_str(), max, min);
            msg_s.have_manip_map = 1; 
            msg_s.dof_value_0 = min;
            msg_s.dof_value_1 = max;
            
            if(!strcmp(msg_s.dof_name.c_str(), "steering_joint")){//!msg_s.dof_name.compare("steering_joint")){
                _lcm->publish("DRIVING_STEERING_ACTUATION_STATUS", &msg_s);
            }
            else if(!strcmp(msg_s.dof_name.c_str(), "gas_joint")){//!msg_s.dof_name.compare(msg_s.dof_name, "gas_joint")){
                _lcm->publish("DRIVING_GAS_ACTUATION_STATUS", &msg_s);
            }
            else if(!strcmp(msg_s.dof_name.c_str(), "brake_joint")){//!msg_s.dof_name.compare("brake_joint")){
                _lcm->publish("DRIVING_BRAKE_ACTUATION_STATUS", &msg_s);
            }
            else if(!strcmp(msg_s.dof_name.c_str(), "hand_brake_joint")){//!msg_s.dof_name.compare("hand_brake_joint")){
                _lcm->publish("DRIVING_HAND_BRAKE_ACTUATION_STATUS", &msg_s);
            }
        }
    }
  }
  
  
  void RobotPlanListener::commit_desired_grasp_state(int64_t utime, std::string &channel, std::vector<int> &received_plan_grasp_indices)
  {
    int num_grasps = received_plan_grasp_indices.size();
    
    std::vector<drc::grasp_transition_state_t> states;
     int ind = received_plan_grasp_indices[0];
     states.push_back(_received_plan.grasps[ind]);
    if(num_grasps>1){
     ind = received_plan_grasp_indices[1];
     states.push_back(_received_plan.grasps[ind]); 
     }
     
    if(num_grasps>2){
      cerr<< "ERROR: More than two grasps for a given transition are actieve. Only one left and one right are physically possible . " << endl;
      return;
    }
     
    //drc::grasp_transition_state_t state = _received_plan.grasps[received_plan_grasp_index];  
    drc::desired_grasp_state_t msg;
    msg.utime = utime;
    msg.robot_name = _robot_name;
    
    msg.object_name = " ";
    msg.geometry_name = " ";
    msg.unique_id = states[0].affordance_uid;
    msg.num_r_joints =0;
    msg.num_l_joints =0;
    msg.l_hand_pose.rotation.w = 1;
    msg.r_hand_pose.rotation.w = 1;
    if(states[0].grasp_type==states[0].LEFT)
    {
      msg.grasp_type = msg.SANDIA_LEFT;
      msg.l_hand_pose = states[0].hand_pose;
      msg.num_l_joints =states[0].num_joints;
      msg.l_joint_name  =states[0].joint_name;
      msg.l_joint_position=states[0].joint_position;
      
      if(num_grasps>1){
          if(states[1].grasp_type==states[1].RIGHT){
            msg.grasp_type = msg.SANDIA_BOTH;
            msg.r_hand_pose = states[1].hand_pose;
            msg.num_r_joints =states[1].num_joints;
            msg.r_joint_name  =states[1].joint_name;
            msg.r_joint_position=states[1].joint_position;
          }
          else
          {
            cerr<< "ERROR: Two grasps for a given transition are both left.  Two left grasps are not physically possible" << endl;
            return;
          }
      }      
    }
    else
    {
      msg.grasp_type = msg.SANDIA_RIGHT;
      msg.r_hand_pose = states[0].hand_pose;
      msg.num_r_joints =states[0].num_joints;
      msg.r_joint_name  =states[0].joint_name;
      msg.r_joint_position=states[0].joint_position;
      if(num_grasps>1){
       if(states[1].grasp_type==states[1].LEFT){
          msg.grasp_type = msg.SANDIA_BOTH;
          msg.l_hand_pose = states[1].hand_pose;
          msg.num_l_joints =states[1].num_joints;
          msg.l_joint_name  =states[1].joint_name;
          msg.l_joint_position=states[1].joint_position;
        }
        else{
            cerr<< "ERROR: Two grasps for a given transition are both right.  Two left grasps are not physically possible" << endl;
            return;
        }
      }  
    }
    msg.power_grasp = states[0].power_grasp;
    if(num_grasps>1)
     msg.power_grasp = (states[0].power_grasp||states[1].power_grasp);
    _lcm->publish(channel, &msg);
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

} //namespace renderer_robot_plan

//      double tic = bot_timestamp_now();
//      double toc = bot_timestamp_now();
//      cout<< "elaspsed "<<(toc-tic)/1000000 << endl;
