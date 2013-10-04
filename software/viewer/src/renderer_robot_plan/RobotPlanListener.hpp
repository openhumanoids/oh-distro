#ifndef RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP
#define RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <visualization_utils/file_access_utils.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/affordance_utils/affordance_seed_utils.hpp>
#include <sys/time.h>
#include <time.h>

namespace renderer_robot_plan 
{

  class RobotPlanListener
  {
    //--------fields
    
   public:
    std::string _robot_name;
    std::string _lhand_ee_name;
    std::string _rhand_ee_name;
   
   private:      
    std::string _urdf_xml_string;   
    
   
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false

    boost::shared_ptr<lcm::LCM> _lcm;    
    
    //get rid of this
    BotViewer *_viewer;

    
    bool _urdf_parsed;
    bool _urdf_subscription_on;
    bool _aprvd_footstep_plan_in_cache;
    bool _is_multi_approve_plan;
    bool _plan_paused; // a manip plan is paused on pressing the pause button, unpaused on pressing play again.
    
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_robot;
    //boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _base_gl_robot;
    //----------------constructor/destructor
    
    int _in_motion_keyframe_index;


  public:
    bool _is_manip_plan;
    bool _is_manip_map;
    int64_t _last_plan_msg_timestamp; 
    RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer, int operation_mode);
    ~RobotPlanListener();
    
   // boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    //std::vector< boost::shared_ptr<visualization_utils::GlKinematicBody> >  _gl_robot_list;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_list;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_keyframe_list;
    std::vector< int64_t >  _keyframe_timestamps;
    std::vector< int >  _breakpoint_indices;
    int _active_breakpoint;
    int _num_breakpoints;
    int _retractable_cycle_counter;
    
    //The following are local in-motion copies that appear on doubleclk of a keyframe and can be moved around via markers
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_left_hand;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_right_hand;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_left_foot;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_right_foot;
    //Pelvis, COM and JointDOF Markers
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_pelvis;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_com;
    //boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_phantom_robot;
    
    
    //-------------message callback
    drc::robot_state_t _last_robotstate_msg;
    drc::robot_plan_t _received_plan;
    drc::robot_plan_w_keyframes_t _received_keyframe_plan;
    
    // previous plan (used for undo button.)
    drc::robot_plan_w_keyframes_t _previous_keyframe_plan; 
    
    drc::aff_indexed_robot_plan_t _received_map;
    drc::footstep_plan_t _received_footstep_plan;
    
    int64_t _controller_utime;
    int8_t _controller_status;
  
    void commit_robot_plan(int64_t utime,std::string &channel);
    void commit_manip_plan(int64_t utime,std::string &channel);
    void commit_compliant_manip_plan(int64_t utime,std::string &channel,int,int,int,int);
    void commit_manip_map(int64_t utime,std::string &channel);
    void commit_footstep_plan(int64_t utime,std::string &channel);    
    void commit_desired_grasp_state(int64_t utime, std::string &channel,std::vector<int> &received_plan_grasp_indices);
    void commit_plan_control(int64_t utime, std::string &channel,bool pause, bool terminate, bool revert);

    void set_in_motion_hands_state(int index)
    {
 
      KDL::Frame T_base_palm,T_world_palm_l,T_world_palm_r,T_world_base_l,T_world_base_r;
     
      T_base_palm = KDL::Frame::Identity();
      T_base_palm.M = KDL::Rotation::RPY(0,-M_PI/2,0);
      
      _gl_robot_keyframe_list[index]->get_link_frame(_lhand_ee_name,T_world_palm_l);
      _gl_robot_keyframe_list[index]->get_link_frame(_rhand_ee_name,T_world_palm_r);

      T_world_base_l = T_world_palm_l*(T_base_palm.Inverse());
      T_world_base_r = T_world_palm_r*(T_base_palm.Inverse());
      
      // Flip marker direction to always point away from the body center.
      double normal, flipped;
      Eigen::Vector3f u_x(1,0,0);
      Eigen::Vector3f u_y(0,1,0);
      Eigen::Vector3f u_hand_to_body;
      u_hand_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_palm_l.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_palm_l.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_palm_l.p[2]; 
      u_hand_to_body.normalize();
      
      normal = acos(u_hand_to_body.dot(u_x));
      flipped = acos(u_hand_to_body.dot(-u_x));
      if(flipped>normal+1e-1) {
        _gl_left_hand->flip_trans_marker_xdir(true);
        }
      else{
       _gl_left_hand->flip_trans_marker_xdir(false);
       }
       
      normal = acos(u_hand_to_body.dot(u_y));
      flipped = acos(u_hand_to_body.dot(-u_y));
      if(flipped>normal+1e-1){
        _gl_left_hand->flip_trans_marker_ydir(true);
        }
      else{
       _gl_left_hand->flip_trans_marker_ydir(false); 
       }
      u_hand_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_palm_r.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_palm_r.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_palm_r.p[2]; 
      u_hand_to_body.normalize();
      
      normal = acos(u_hand_to_body.dot(u_x));
      flipped = acos(u_hand_to_body.dot(-u_x));
      if(flipped>normal+1e-1){
        _gl_right_hand->flip_trans_marker_xdir(true);
        }
      else{
       _gl_right_hand->flip_trans_marker_xdir(false);
       }
      normal = acos(u_hand_to_body.dot(u_y));
      flipped = acos(u_hand_to_body.dot(-u_y));
      if(flipped>normal+1e-1){
        _gl_right_hand->flip_trans_marker_ydir(true);
        }
      else{
       _gl_right_hand->flip_trans_marker_ydir(false);  
       }
       
      std::map<std::string, double> jointpos_l;
      jointpos_l=_gl_left_hand->_current_jointpos;
      _gl_left_hand->set_state(T_world_base_l,jointpos_l);
      _gl_left_hand->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
       std::map<std::string, double> jointpos_r;
      jointpos_r = _gl_right_hand->_current_jointpos;
      _gl_right_hand->set_state(T_world_base_r,jointpos_r);    
      _gl_right_hand->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
      _in_motion_keyframe_index = index;
    
    };
    
    void set_in_motion_feet_state(int index)
    {
 
      KDL::Frame T_world_foot_l,T_world_foot_r;
     
      _gl_robot_keyframe_list[index]->get_link_frame("l_foot",T_world_foot_l);
      _gl_robot_keyframe_list[index]->get_link_frame("r_foot",T_world_foot_r);

      // Flip marker direction to always point away from the body center.

      double normal, flipped;
      Eigen::Vector3f u_x(1,0,0);
      Eigen::Vector3f u_y(0,1,0);
      Eigen::Vector3f u_foot_to_body;
      u_foot_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_foot_l.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_foot_l.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_foot_l.p[2]; 
      u_foot_to_body.normalize();
      
      normal = acos(u_foot_to_body.dot(u_x));
      flipped = acos(u_foot_to_body.dot(-u_x));
      if(flipped>normal+1e-1) 
      {
        _gl_left_foot->flip_trans_marker_xdir(true);
      }
      else
      {
        _gl_left_foot->flip_trans_marker_xdir(false);
      }
       
      normal = acos(u_foot_to_body.dot(u_y));
      flipped = acos(u_foot_to_body.dot(-u_y));
      if(flipped>normal+1e-1)
      {
        _gl_left_foot->flip_trans_marker_ydir(true);
      }
      else
      {
        _gl_left_foot->flip_trans_marker_ydir(false); 
      }
      u_foot_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_foot_r.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_foot_r.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_foot_r.p[2]; 
      u_foot_to_body.normalize();
      
      normal = acos(u_foot_to_body.dot(u_x));
      flipped = acos(u_foot_to_body.dot(-u_x));
      if(flipped>normal+1e-1){
        _gl_right_foot->flip_trans_marker_xdir(true);
        }
      else{
       _gl_right_foot->flip_trans_marker_xdir(false);
       }
      normal = acos(u_foot_to_body.dot(u_y));
      flipped = acos(u_foot_to_body.dot(-u_y));
      if(flipped>normal+1e-1)
      {
        _gl_right_foot->flip_trans_marker_ydir(true);
      }
      else
      {
        _gl_right_foot->flip_trans_marker_ydir(false);  
      }
       
      std::map<std::string, double> jointpos_l;
      jointpos_l=_gl_left_foot->_current_jointpos;
      _gl_left_foot->set_state(T_world_foot_l,jointpos_l);
      _gl_left_foot->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
       std::map<std::string, double> jointpos_r;
      jointpos_r = _gl_right_foot->_current_jointpos;
      _gl_right_foot->set_state(T_world_foot_r,jointpos_r);    
      _gl_right_foot->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
      _in_motion_keyframe_index = index;
    
    };    
    
    void set_in_motion_pelvis_state(int index)
    {
 
      KDL::Frame T_world_pelvis;     
      _gl_robot_keyframe_list[index]->get_link_frame("pelvis",T_world_pelvis);

      // Flip marker direction to always point away from the body center.
      double normal, flipped;
      Eigen::Vector3f u_x(1,0,0);
      Eigen::Vector3f u_y(0,1,0);
      Eigen::Vector3f u_pelvis_to_body;
      u_pelvis_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_pelvis.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_pelvis.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_pelvis.p[2]; 
      u_pelvis_to_body.normalize();
      
      if(u_pelvis_to_body.norm()>0)
      {
        normal = acos(u_pelvis_to_body.dot(u_x));
        flipped = acos(u_pelvis_to_body.dot(-u_x));
        if(flipped>normal+1e-1) 
        {
          _gl_pelvis->flip_trans_marker_xdir(true);
        }
        else
        {
          _gl_pelvis->flip_trans_marker_xdir(false);
        }
         
        normal = acos(u_pelvis_to_body.dot(u_y));
        flipped = acos(u_pelvis_to_body.dot(-u_y));
        if(flipped>normal+1e-1)
        {
          _gl_pelvis->flip_trans_marker_ydir(true);
        }
        else
        {
          _gl_pelvis->flip_trans_marker_ydir(false); 
        }
      }
      else
      {
        _gl_pelvis->flip_trans_marker_xdir(false); 
        _gl_pelvis->flip_trans_marker_ydir(false); 
      }
      _gl_pelvis->flip_trans_marker_zdir(true);
      std::map<std::string, double> jointpos;
      jointpos=_gl_pelvis->_current_jointpos;
      _gl_pelvis->set_state(T_world_pelvis,jointpos);
      _gl_pelvis->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
    
      _in_motion_keyframe_index = index;
    };        
    
    void set_in_motion_com_state(int index)
    {
 
      KDL::Frame T_world_com; 
      _gl_robot_keyframe_list[index]->get_com_frame(T_world_com);
      
      // Flip marker direction to always point away from the body center.
      double normal, flipped;
      Eigen::Vector3f u_x(1,0,0);
      Eigen::Vector3f u_y(0,1,0);
      Eigen::Vector3f u_com_to_body;
      u_com_to_body << _gl_robot_keyframe_list[index]->_T_world_body.p[0]-T_world_com.p[0],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[1]-T_world_com.p[1],
                           _gl_robot_keyframe_list[index]->_T_world_body.p[2]-T_world_com.p[2]; 
      u_com_to_body.normalize();
      
      if(u_com_to_body.norm()>0)
      {
        normal = acos(u_com_to_body.dot(u_x));
        flipped = acos(u_com_to_body.dot(-u_x));
        if(flipped>normal+1e-1) 
        {
          _gl_com->flip_trans_marker_xdir(true);
        }
        else
        {
          _gl_com->flip_trans_marker_xdir(false);
        }
         
        normal = acos(u_com_to_body.dot(u_y));
        flipped = acos(u_com_to_body.dot(-u_y));
        if(flipped>normal+1e-1)
        {
          _gl_com->flip_trans_marker_ydir(true);
        }
        else
        {
          _gl_com->flip_trans_marker_ydir(false); 
        }
      }
      else
      {
        _gl_com->flip_trans_marker_xdir(false); 
        _gl_com->flip_trans_marker_ydir(false); 
      }
      _gl_com->flip_trans_marker_zdir(true);
      std::map<std::string, double> jointpos;
      jointpos=_gl_com->_current_jointpos;
      _gl_com->set_state(T_world_com,jointpos);
      _gl_com->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
    
      _in_motion_keyframe_index = index;
    };           
              
    
    
    
    bool is_in_motion(int index) {  
        return (index==_in_motion_keyframe_index);
    };
    
		bool is_walking_plan()
    {
		    return (_aprvd_footstep_plan_in_cache&&(!_is_manip_map)&&(!_is_manip_plan));
		};
		
		bool is_manip_plan()
    {
		    return _is_manip_plan;
		};
		
		bool is_manip_map()
    {
		    return _is_manip_map;
		};
		
		bool is_multi_approval_plan()
    {
		    return _is_multi_approve_plan;
		};
		
		bool is_plan_paused()
		{
		  return _plan_paused;
		};
		
		void purge_current_plan(){
		    _gl_robot_list.clear();
        if(_is_manip_plan){
        _gl_robot_keyframe_list.clear();
        _gl_left_hand->enable_bodypose_adjustment(false); 
        _gl_right_hand->enable_bodypose_adjustment(false);
        _gl_left_foot->enable_bodypose_adjustment(false); 
        _gl_right_foot->enable_bodypose_adjustment(false);
        _gl_pelvis->enable_bodypose_adjustment(false); 
        _gl_com->enable_bodypose_adjustment(false);
        _is_manip_plan = false;
        }
        _is_manip_map = false;
		};	
		
		void activate_walking_plan()
    {
		  _is_manip_plan =false;
      _is_manip_map =false; 
      deactivate_multi_approval();  
		};
		
		bool activate_manip_plan()
    {
      _is_manip_plan =true;
      _is_manip_map =false;
      if(_aprvd_footstep_plan_in_cache){
       _aprvd_footstep_plan_in_cache=false;
      }
		};
		
		bool activate_manip_map()
    {
      _is_manip_plan =false;
      _is_manip_map =true;
      if(_aprvd_footstep_plan_in_cache)
       _aprvd_footstep_plan_in_cache=false;
      deactivate_multi_approval(); 
		};
		
		bool activate_multi_approval()
    {
      _is_multi_approve_plan =true;
      _active_breakpoint=0;
      _retractable_cycle_counter = 0;
		};
		
		bool deactivate_multi_approval()
    {
      _is_multi_approve_plan =false;
      _active_breakpoint=0;
      _retractable_cycle_counter = 0;
		};
    
    
    int64_t get_keyframe_timestamp(int index) {
        return _keyframe_timestamps[index];
    };
    
  private:
   void handleRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_plan_t* msg);
	 void handleManipPlanMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_plan_w_keyframes_t* msg);
   void handleAffIndexedRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::aff_indexed_robot_plan_t* msg);		      
   void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg);
   void handleAprvFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
						 const drc::footstep_plan_t* msg);    
	 void handleCanFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
						 const drc::footstep_plan_t* msg);  				 
   void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);	
   void handleControllerStatusMsg(const lcm::ReceiveBuffer* rbuf,
                                                 const string& chan, 
                                                 const drc::controller_status_t* msg);
						 
  void appendHandStatesToStateMsg(const drc::robot_plan_w_keyframes_t* msg,drc::robot_state_t *state_msg,bool append_currenthandstate)	 
  {
    // Merge in grasp state transitions into the plan states.
    if(msg->num_grasp_transitions>0)
    {
      size_t k_max = 0;
      for(size_t k=0;k<msg->num_grasp_transitions;k++)
      {
        // get the relevant transition state time from msg->grasps[k].utime
        if(state_msg->utime >= msg->grasps[k].utime)
           k_max = std::max(k_max,k);
      }
      
      std::vector<int> kmax_inds;
      //_received_plan.grasps[k].utime can be non-unique upto 2 (getting both inds)
       for(size_t k=0;k<msg->num_grasp_transitions;k++)
      {
        if(msg->grasps[k].utime==msg->grasps[k_max].utime)
            kmax_inds.push_back(k);
      }

      for(size_t j=0;j< kmax_inds.size();j++)
      {
        k_max= kmax_inds[j];
        for(size_t i=0;i< msg->grasps[k_max].num_joints;i++)
        {
           state_msg->num_joints++;
           state_msg->joint_name.push_back(msg->grasps[k_max].joint_name[i]);  
           state_msg->joint_position.push_back(msg->grasps[k_max].joint_position[i]);  
           state_msg->joint_velocity.push_back(0);
           state_msg->joint_effort.push_back(0);
         }// end for  
       }     
       
    }// end if	
    else if(append_currenthandstate) 
    {
        std::vector<std::string> joint_names = _gl_left_hand->get_joint_names();
        for(size_t i=0;i< joint_names.size();i++)
        {
            std::vector<std::string>::const_iterator found;
            found = std::find(_last_robotstate_msg.joint_name.begin(), _last_robotstate_msg.joint_name.end(), joint_names[i]);
            if (found != _last_robotstate_msg.joint_name.end()) {
              unsigned int index = found - _last_robotstate_msg.joint_name.begin();
              state_msg->num_joints++;
              state_msg->joint_name.push_back(joint_names[i]);   
              state_msg->joint_position.push_back(_last_robotstate_msg.joint_position[index]); 
              state_msg->joint_velocity.push_back(0);
              state_msg->joint_effort.push_back(0);
            }
 
         }// end for  
         
        joint_names = _gl_right_hand->get_joint_names();
        for(size_t i=0;i< joint_names.size();i++)
        {

            std::vector<std::string>::const_iterator found;
            found = std::find(_last_robotstate_msg.joint_name.begin(), _last_robotstate_msg.joint_name.end(), joint_names[i]);
            if (found != _last_robotstate_msg.joint_name.end()) {
              unsigned int index = found - _last_robotstate_msg.joint_name.begin();
              state_msg->num_joints++;
              state_msg->joint_name.push_back(joint_names[i]);   
              state_msg->joint_position.push_back(_last_robotstate_msg.joint_position[index]); 
              state_msg->joint_velocity.push_back(0);
              state_msg->joint_effort.push_back(0);
            }
         }// end for 
    }
  };
			      
  bool load_hand_urdfs(bool _is_left_sandia, bool _is_right_sandia, std::string &_left_hand_urdf_xml_string,std::string &_right_hand_urdf_xml_string);
  bool load_foot_urdfs(std::string &_left_foot_urdf_xml_string,std::string &_right_foot_urdf_xml_string);
  bool load_misc_marker_urdfs(std::string &_pelvis_urdf_xml_string,std::string &_com_urdf_xml_string);
  
  public:
  //-------------------------------
  void prepareActivePlanForStorage(KDL::Frame &T_world_aff, 
                                   std::string &type,
                                   std::vector<std::string> &stateframe_ids,
                                   std::vector< std::vector<double> > &stateframe_values,
                                   std::vector<std::string> &graspframe_ids,
                                   std::vector< std::vector<double> > &graspframe_values)
  { 
    if(is_manip_plan()){// uses _received_keyframe_plan
      type = "manip";
      visualization_utils::prepareKeyframePlanForStorage(T_world_aff,_received_keyframe_plan,stateframe_ids,stateframe_values,graspframe_ids,graspframe_values);
    }
    else if(is_walking_plan()){// uses _received_plan
      type = "walking"; 
      
      drc::robot_plan_w_keyframes_t msg;
      msg.utime = _received_plan.utime;
      msg.robot_name = _received_plan.robot_name;
      msg.num_states = _received_plan.num_states;
      for (uint i = 0; i <(uint)msg.num_states; i++)
      {
      msg.is_keyframe[i] = false;
      msg.is_breakpoint[i] = false;
      }
      msg.plan = _received_plan.plan;
      msg.num_bytes = _received_plan.num_bytes;
      msg.matlab_data = _received_plan.matlab_data;
      msg.num_grasp_transitions = _received_plan.num_grasp_transitions;  
      visualization_utils::prepareKeyframePlanForStorage(T_world_aff,msg,stateframe_ids,stateframe_values,graspframe_ids,graspframe_values);
    }
    else
      type= "unknown";
  }; // end method
  
  
  void setPlanFromStorage(KDL::Frame &T_world_aff, 
                                   std::string &type,
                                   std::vector<std::string> &stateframe_ids,
                                   std::vector< std::vector<double> > &stateframe_values,
                                   std::vector<std::string> &graspframe_ids,
                                   std::vector< std::vector<double> > &graspframe_values)
  {

    drc::robot_plan_w_keyframes_t msg;                              
    visualization_utils::decodeKeyframePlanFromStorage(T_world_aff,stateframe_ids,stateframe_values,graspframe_ids,graspframe_values,msg);

    if(type == "manip")
    {
      handleManipPlanMsg(NULL," ",&msg);
      std::string channel = "STORED_ROBOT_PLAN";
      _lcm->publish(channel, &msg);  // Msg used to update plan cache in KEYFRAME ADJUSTMENT ENGINE. 
    }
    else if(type == "walking")
    {
      drc::robot_plan_t msg_subset;  
      msg_subset.left_arm_control_type = msg_subset.NONE;
      msg_subset.right_arm_control_type = msg_subset.NONE;
      msg_subset.left_leg_control_type = msg_subset.NONE;
      msg_subset.right_leg_control_type = msg_subset.NONE;
      msg_subset.utime = msg.utime;
      msg_subset.robot_name = msg.robot_name;
      msg_subset.num_states = msg.num_states;
      msg_subset.plan = msg.plan;
      msg_subset.num_bytes = msg.num_bytes;
      msg_subset.matlab_data = msg.matlab_data;
      msg_subset.num_grasp_transitions = msg.num_grasp_transitions;
      if(msg.num_grasp_transitions>0)
        msg_subset.grasps = msg.grasps;
      handleRobotPlanMsg(NULL," ",&msg_subset);
      //std::string channel = "STORED_ROBOT_PLAN";
      // Msg used to update plan cache in KEYFRAME ADJUSTMENT ENGINE. (Walking plan are currently not adjustable)
      //_lcm->publish(channel, &msg_subset);  
      // TODO: also publish a SEEDED footstep plan? piping with robin? 
    }
    else
     cout <<"Unknown Plan type in RobotPlanListener::setPlanFromStorage\n";
  };
  
  void setManipPlanFromBackUp()
  {
    if(!is_manip_plan())
      return;
  
    drc::robot_plan_w_keyframes_t msg;  
    msg = _previous_keyframe_plan;
    if(msg.num_states>0)
    {
     _received_keyframe_plan = _previous_keyframe_plan;
     handleManipPlanMsg(NULL," ",&msg);
     std::string channel = "STORED_ROBOT_PLAN";
     _lcm->publish(channel, &msg);  // Msg used to update plan cache in KEYFRAME ADJUSTMENT ENGINE. 
     
     drc::system_status_t stat_msg;
     stat_msg.utime = bot_timestamp_now();
     stat_msg.system = 0x03;
     stat_msg.value = "Undoing replan or plan adjustment." ;
     _lcm->publish("SYSTEM_STATUS", &stat_msg);
    }
  };
    
 
}; //class RobotPlanListener
  

} //namespace renderer_robot_plan


#endif //RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP
