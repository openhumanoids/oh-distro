// Maintains a list of sticky feet generated from a footstep planner published on CANDIDATE_FOOTSTEP_PLAN.
// Footstep planner generates a ee_goal_sequence on receipt of a nav_goal.
// TODO: Support to handle N FOOTSTEP plans? currently one list is maintained.

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "FootStepPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_sticky_feet
{
  //==================constructor / destructor
  
//  FootStepPlanListener::FootStepPlanListener(RendererStickyFeet* parent_renderer):
//    _parent_renderer(parent_renderer)
   
  FootStepPlanListener::FootStepPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer):
    _lcm(lcm),_viewer(viewer)
  {
     //_collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());
    //lcm ok?
    
    //_lcm = _parent_renderer->lcm;
    if(!_lcm->good())
    {
      cerr << "\nLCM Not Good: Robot FootStepPlan Handler" << endl;
      return;
    }
      in_motion_footstep_id=-1;

     _last_plan_approved = false;
     _waiting_for_new_plan = false;
     _left_foot_name ="l_foot";
     _right_foot_name = "r_foot";
    if(!load_foot_urdfs())
      return;
      
      _base_gl_stickyfoot_left =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_left_urdf_xml_string));     
      _base_gl_stickyfoot_right =  shared_ptr<GlKinematicBody>(new GlKinematicBody(_right_urdf_xml_string));
 
      std::map<std::string, double> jointpos_in;
      jointpos_in =  _base_gl_stickyfoot_left->_current_jointpos;
      _base_gl_stickyfoot_left->set_state(_base_gl_stickyfoot_left->_T_world_body,jointpos_in); // set to initialized values.
      jointpos_in.clear();
      jointpos_in =  _base_gl_stickyfoot_right->_current_jointpos;
      _base_gl_stickyfoot_right->set_state(_base_gl_stickyfoot_left->_T_world_body,jointpos_in); // set to initialized values.
      
      
      
      
      Eigen::Vector3f whole_body_span;
      Eigen::Vector3f offset;
      MeshStruct mesh_struct;
      bool val;
      _base_gl_stickyfoot_left->get_whole_body_span_dims(whole_body_span,offset);
      
      val =_base_gl_stickyfoot_left->get_mesh_struct("l_talus_0", mesh_struct);
      _T_bodyframe_groundframe_left = KDL::Frame::Identity();
      _T_bodyframe_groundframe_left.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);    
      
      _base_gl_stickyfoot_right->get_whole_body_span_dims(whole_body_span,offset);
      val = _base_gl_stickyfoot_right->get_mesh_struct("r_talus_0", mesh_struct); 
     _T_bodyframe_groundframe_right = KDL::Frame::Identity();
     _T_bodyframe_groundframe_right.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);
     
//     Eigen::Vector3f whole_body_span;
//     Eigen::Vector3f offset;
//     MeshStruct mesh_struct;
//      bool val;
//       
//     _base_gl_stickyfoot_left->get_whole_body_span_dims(whole_body_span,offset);
//     val = _base_gl_stickyfoot_left->get_mesh_struct("l_foot_0", mesh_struct);
//    
// 
//      val =_base_gl_stickyfoot_left->get_mesh_struct("l_talus_0", mesh_struct);
//     _T_bodyframe_meshframe_left = KDL::Frame::Identity();
//     _T_bodyframe_meshframe_left.p[0] =   -mesh_struct.offset_x;
//     _T_bodyframe_meshframe_left.p[1] =   -mesh_struct.offset_y;
//     _T_bodyframe_meshframe_left.p[2] =   -mesh_struct.offset_z;
//    
//     _T_bodyframe_groundframe_left = KDL::Frame::Identity();
//     _T_bodyframe_groundframe_left.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);
// 
//     
//     _base_gl_stickyfoot_right->get_whole_body_span_dims(whole_body_span,offset);
//     val = _base_gl_stickyfoot_right->get_mesh_struct("r_talus_0", mesh_struct);
//     _T_bodyframe_meshframe_right = KDL::Frame::Identity();
//     _T_bodyframe_meshframe_right.p[0] =   -mesh_struct.offset_x;
//     _T_bodyframe_meshframe_right.p[1] =   -mesh_struct.offset_y;
//     _T_bodyframe_meshframe_right.p[2] =   -mesh_struct.offset_z;
//    
//     _T_bodyframe_groundframe_right = KDL::Frame::Identity();
//     _T_bodyframe_groundframe_right.p[2] = whole_body_span[2]-0.5*(mesh_struct.span_z);
     
     

    lcm->subscribe("CANDIDATE_FOOTSTEP_PLAN", &renderer_sticky_feet::FootStepPlanListener::handleFootStepPlanMsg, this); //&this ?
    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize   

  }
  
  FootStepPlanListener::~FootStepPlanListener() {
   // _collision_detector->clear_collision_objects();
  }
  

  //=============message callbacks

void FootStepPlanListener::handleFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::footstep_plan_t* msg)						 
  {
   cout << "\n handleFootStepPlanMsg: Footstep plan received" << endl;
  
   // 0. Make Local copy to later output
    revieved_plan_ = *msg;
    _last_plan_approved = false;

    _robot_name = msg->robot_name;
  	int num_steps = 0;
		num_steps = msg->num_steps;   

    if(_waiting_for_new_plan && !msg->is_new_plan) {
      bot_viewer_request_redraw(_viewer);
      return;
    }
    _waiting_for_new_plan = false;
		if(msg->is_new_plan)
    {
      // clear old motion copy
      _gl_in_motion_copy.reset();
      in_motion_footstep_id=-1;
    }
    
		int old_in_motion_footstep_id=in_motion_footstep_id; 		

    //clear stored data
    _gl_planned_stickyfeet_list.clear();
    _planned_stickyfeet_info_list.clear();
    _gl_planned_stickyfeet_timestamps.clear();
    _gl_planned_stickyfeet_ids.clear();
    
    

    
    //cout << "utime: "<< msg->utime<< endl;
   
    for (uint i = 0; i <(uint)num_steps; i++)
    {
     
    
      drc::footstep_goal_t goal_msg  = msg->footstep_goals[i]; 
      _gl_planned_stickyfeet_timestamps.push_back(goal_msg.step_time);
      _gl_planned_stickyfeet_ids.push_back(goal_msg.id);
      
      
      // cout << "id: "<< goal_msg.id<< endl;
      // cout << "timestamp: "<< goal_msg.step_time<< endl;
      std::stringstream oss;
      oss << msg->robot_name << "_"<< goal_msg.id;
     // cout << "names: "<< oss.str() << endl;

      // KDL::Frame T_worldframe_meshframe;
      KDL::Frame T_worldframe_footframe;
      transformLCMToKDL(goal_msg.pos, T_worldframe_footframe);
      
      StickyFeetInfoStruct info;
       bool is_constrained = ((goal_msg.fixed_x)||(goal_msg.fixed_y)||(goal_msg.fixed_z)||(goal_msg.fixed_roll)||(goal_msg.fixed_pitch)||(goal_msg.fixed_yaw));
      if(!goal_msg.is_right_foot)
      {

        // KDL::Frame T_worldframe_bodyframe =  T_worldframe_meshframe*_T_bodyframe_meshframe_left.Inverse();
        // KDL::Frame T_worldframe_groundframe = T_worldframe_bodyframe*_T_bodyframe_groundframe_left;
      
        shared_ptr<InteractableGlKinematicBody>  new_object_ptr(new InteractableGlKinematicBody(*_base_gl_stickyfoot_left,true,oss.str()));
        _gl_planned_stickyfeet_list.push_back(new_object_ptr);
         info.foot_type = FootStepPlanListener::LEFT;
         info.is_fixed=is_constrained;
        _planned_stickyfeet_info_list.push_back(info);
        _gl_planned_stickyfeet_list[i]->enable_whole_body_selection(true);
         std::map<std::string, double> jointpos_in; 
         jointpos_in =  _gl_planned_stickyfeet_list[i]->_current_jointpos;

        _gl_planned_stickyfeet_list[i]->set_state(T_worldframe_footframe,jointpos_in);       
        _gl_planned_stickyfeet_list[i]->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::TWO_D);
      }
      else
      {
      

        // KDL::Frame T_worldframe_bodyframe =  T_worldframe_meshframe*_T_bodyframe_meshframe_right.Inverse();
        // KDL::Frame T_worldframe_groundframe = T_worldframe_bodyframe*_T_bodyframe_groundframe_right;

        shared_ptr<InteractableGlKinematicBody>  new_object_ptr(new InteractableGlKinematicBody(*_base_gl_stickyfoot_right,true,oss.str()));
        _gl_planned_stickyfeet_list.push_back(new_object_ptr);
         info.foot_type = FootStepPlanListener::RIGHT;
         info.is_fixed=is_constrained;
        _planned_stickyfeet_info_list.push_back(info);
        _gl_planned_stickyfeet_list[i]->enable_whole_body_selection(true); 
         std::map<std::string, double> jointpos_in; 
         jointpos_in =  _gl_planned_stickyfeet_list[i]->_current_jointpos;
        
        _gl_planned_stickyfeet_list[i]->set_state(T_worldframe_footframe,jointpos_in);
        _gl_planned_stickyfeet_list[i]->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::TWO_D);
      }  
      
      
//      if((old_in_motion_footstep_id!=-1)&&(old_in_motion_footstep_id == goal_msg.id))
//      {      
//        _gl_planned_stickyfeet_list[i]->enable_bodypose_adjustment(true); // make the current in_motion_footstep persistent across candidate plan publishes
//      }
      
     

    }//end for num of goals;

    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
    bot_viewer_request_redraw(_viewer);
  } // end handleMessage
  
  
//------------------------------------------------------------------------------ 

  bool  FootStepPlanListener::load_foot_urdfs()
  {   
  

    string urdf_models_path = string(getModelsPath()) + "/mit_gazebo_models/mit_robot_feet/"; // getModelsPath gives /drc/software/build/models/
    cout << "searching for foot urdf files in: "<< (urdf_models_path) << endl;
    vector<string> urdf_files = vector<string>();
    int res = get_URDF_filenames_from_dir(urdf_models_path, urdf_files);  
  
     
    if(res==0)  //urdf found
    {
       cout << "found " << urdf_files.size() << " " << urdf_files[0] << " " << urdf_files[1] << " files"<< endl;
    }
    else{
      cerr << "ERROR: no urdf files found in: "<< (urdf_models_path) << endl;
      return false;
    } 
        
    

    std::vector<std::string>::const_iterator found;
    found = std::find (urdf_files.begin(), urdf_files.end(), _left_foot_name); 
    if(found !=  urdf_files.end()) 
    {
      unsigned int index = found - urdf_files.begin();
      std::stringstream oss;
      oss << urdf_models_path << urdf_files[index] << ".urdf" ;
      get_xmlstring_from_file(oss.str(), _left_urdf_xml_string);
    }
    else
    {
      cerr <<"ERROR: " << _left_foot_name  << ".urdf not found"<< endl;
      return false;
    }

    found = std::find (urdf_files.begin(), urdf_files.end(), _right_foot_name);  
    if(found !=  urdf_files.end()) 
    {
      unsigned int index = found - urdf_files.begin();
      std::stringstream oss;
      oss << urdf_models_path << urdf_files[index] << ".urdf" ;
      get_xmlstring_from_file(oss.str(), _right_urdf_xml_string);
    }
    else
    {
      cerr <<"ERROR:" << _right_foot_name  << ".urdf not found"<< endl;
      return false;
    }
    return true;
  }

 void FootStepPlanListener::commit_footstep_plan(int64_t utime,string &channel)
 {
    drc::footstep_plan_t msg = revieved_plan_;
    
    int64_t old_utime = msg.utime;
    msg.utime = utime;
    int num_steps = 0;
		num_steps = msg.num_steps;  
     for (uint i = 0; i <(uint)num_steps; i++)
    {
        drc::footstep_goal_t goal_msg  = msg.footstep_goals[i];
        
       // KDL::Frame T_worldframe_groundframe = _gl_planned_stickyfeet_list[i]->_T_world_body;
       KDL::Frame T_worldframe_footframe = _gl_planned_stickyfeet_list[i]->_T_world_body;
       goal_msg.is_right_foot = (_planned_stickyfeet_info_list[i].foot_type== FootStepPlanListener::RIGHT);
    
      // if(!goal_msg.is_right_foot)
      // {
      //   T_worldframe_meshframe =  T_worldframe_groundframe*(_T_bodyframe_groundframe_left.Inverse())*_T_bodyframe_meshframe_left;
      // }
      // else
      // {
      //   T_worldframe_meshframe =  T_worldframe_groundframe*(_T_bodyframe_groundframe_right.Inverse())*_T_bodyframe_meshframe_right;
      // } 
      transformKDLToLCM(T_worldframe_footframe,goal_msg.pos); 
      goal_msg.step_time+=utime-old_utime;
      msg.footstep_goals[i] =  goal_msg;
      // msg.goal_times[i]+=utime-old_utime;
    }
 
   _lcm->publish(channel, &msg);
 
 }
} //namespace renderer_sticky_feet

