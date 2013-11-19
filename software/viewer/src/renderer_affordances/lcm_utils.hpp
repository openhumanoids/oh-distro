#ifndef RENDERER_AFFORDANCES_LCM_UTILS_HPP
#define RENDERER_AFFORDANCES_LCM_UTILS_HPP
#include "renderer_affordances.hpp"
#include <visualization_utils/affordance_utils/affordance_lcm_utils.hpp>
#include <visualization_utils/stickyhand_utils/sticky_hand_lcm_utils.hpp>
#include <visualization_utils/stickyfoot_utils/sticky_foot_lcm_utils.hpp>

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
using namespace renderer_affordances;


namespace renderer_affordances_lcm_utils
{

    //---------------------------------------------------------------------------------------------------- 
    static void publish_aff_indexed_traj_opt_constraint(const string& channel, map<string, vector<KDL::Frame> > &ee_frames_map, map<string, vector<drc::affordance_index_t> > &ee_frame_affindices_map,  void *user)
    {
        RendererAffordances *self = (RendererAffordances*) user;
        drc::aff_indexed_traj_opt_constraint_t trajmsg;
        bool unique_ee_occurances=true;
        get_aff_indexed_traj_opt_constraint(self->last_state_msg_timestamp,self->robot_name,unique_ee_occurances,ee_frames_map,ee_frame_affindices_map,trajmsg);
        self->lcm->publish(channel, &trajmsg);
    }
    //----------------------------------------------------------------------------------------------------  
     static void publish_partial_grasp_state_for_execution( StickyHandStruc &sticky_hand_struc,string ee_name,string robot_name, string channel,KDL::Frame &T_world_geometry, 
                                                           int grasp_state, void *user)
    {
        //grasp state = 0 - open
        //grasp state = 1 - partial 
        //grasp state = 2 - full grasp 
        RendererAffordances *self = (RendererAffordances*) user;
        drc::desired_grasp_state_t msg;       
        msg=get_partial_grasp_state(self->last_state_msg_timestamp,robot_name,sticky_hand_struc,ee_name,T_world_geometry,grasp_state);
        // Publish the message 
        self->lcm->publish(channel, &msg);
    }
   //----------------------------------------------------------------------------------------------------
    // one traj_opt_constraint
    // for N ee's and K timesteps. 

    static void publish_traj_opt_constraint(const string& channel, bool unique_ee_occurances,
                                            map<string, vector<KDL::Frame> > &ee_frames_map,
                                            map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                            map<string, vector<double> > &joint_pos_map,
                                            map<string, vector<int64_t> > &joint_pos_timestamps_map,  void *user)
    {
        RendererAffordances *self = (RendererAffordances*) user;
        drc::traj_opt_constraint_t trajmsg;
        get_traj_opt_constraint(self->last_state_msg_timestamp,self->robot_name,unique_ee_occurances,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map,trajmsg);
        self->lcm->publish(channel, &trajmsg);
    }
    //----------------------------------------------------------------------------------------------------
    static void publish_eegoal_to_sticky_hand(boost::shared_ptr<lcm::LCM> &_lcm, StickyHandStruc &sticky_hand_struc,string ee_name, string channel,KDL::Frame &T_world_geometry,bool reach_flag)
    {
        drc::ee_goal_t goalmsg;
        // populate message

        goalmsg=get_eegoal_to_sticky_hand(sticky_hand_struc, "atlas","pelvis",ee_name,T_world_geometry,reach_flag);
        // Publish the message
        _lcm->publish(channel, &goalmsg);
    }  

    //----------------------------------------------------------------------------------------------------
    static void publish_ee_goal_to_gaze(boost::shared_ptr<lcm::LCM> &_lcm, string ee_name, string channel,BotTrans &ee_to_local)
    {
        drc::ee_goal_t goalmsg;
        goalmsg.robot_name = "atlas";
        goalmsg.root_name = "pelvis";
        goalmsg.ee_name = ee_name;

        goalmsg.ee_goal_pos.translation.x = ee_to_local.trans_vec[0];
        goalmsg.ee_goal_pos.translation.y = ee_to_local.trans_vec[1];
        goalmsg.ee_goal_pos.translation.z = ee_to_local.trans_vec[2];

        //note ***** - if he is using KDL Quaternions - then the convention is different than bottrans 
        goalmsg.ee_goal_pos.rotation.x = ee_to_local.rot_quat[1];
        goalmsg.ee_goal_pos.rotation.y = ee_to_local.rot_quat[2];
        goalmsg.ee_goal_pos.rotation.z = ee_to_local.rot_quat[3];
        goalmsg.ee_goal_pos.rotation.w = ee_to_local.rot_quat[0];

        goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
        goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
        goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
        goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
        goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
        goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

        goalmsg.num_chain_joints  = 0; //sticky_hand_struc.joint_name.size();
        // No specified posture bias
        goalmsg.use_posture_bias  = false;
        goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
        goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
        /*for(int i = 0; i < goalmsg.num_chain_joints; i++){
          goalmsg.joint_posture_bias[i]=0;//sticky_hand_struc.joint_position[i];
          goalmsg.chain_joint_names[i]= sticky_hand_struc.joint_name[i];
          }*/

        // Publish the message
        goalmsg.halt_ee_controller = false;

        _lcm->publish(channel, &goalmsg);
    }

    //----------------------------------------------------------------------------------------------------   
    static void publish_desired_hand_motion(StickyHandStruc &sticky_hand_struc, string ee_name, string channel,  void *user)
    {
        RendererAffordances *self = (RendererAffordances*) user;
        
        typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
        object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(sticky_hand_struc.object_name));
        KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;
        drc::traj_opt_constraint_t trajmsg;
        trajmsg = get_desired_hand_motion_traj_constraint(self->last_state_msg_timestamp,self->robot_name,T_world_object,sticky_hand_struc,ee_name);         
        self->lcm->publish(channel, &trajmsg);
    }
  
    //----------------------------------------------------------------------------------------------------
    static void publish_grasp_state_for_execution( StickyHandStruc &sticky_hand_struc,string ee_name,string robot_name, string channel,KDL::Frame &T_world_geometry,  bool grasp_flag,bool power_flag,bool squeeze_flag, double squeeze_amount, void *user)
    {
        RendererAffordances *self = (RendererAffordances*) user;
        drc::desired_grasp_state_t msg;
        msg=get_grasp_state(self->last_state_msg_timestamp,robot_name,sticky_hand_struc,ee_name,T_world_geometry,grasp_flag,power_flag,squeeze_flag,squeeze_amount);
        // Publish the message 
        self->lcm->publish(channel, &msg);
    }
  
    //----------------------------------------------------------------------------------------------------
    static void publish_eegoal_to_sticky_foot(boost::shared_ptr<lcm::LCM> &_lcm, StickyFootStruc &sticky_foot_struc,
                                              string ee_name, string channel,KDL::Frame &T_world_geometry,bool reach_flag)
    {
        drc::ee_goal_t goalmsg;
        // populate message
        goalmsg=get_eegoal_to_stickyfoot(sticky_foot_struc, "atlas","pelvis",ee_name,T_world_geometry,reach_flag);
        // Publish the message
        _lcm->publish(channel, &goalmsg);
    }  
    //----------------------------------------------------------------------------------------------------   
    static void publish_desired_foot_motion(StickyFootStruc &sticky_foot_struc, string ee_name, string channel,  void *user)
    {
        RendererAffordances *self = (RendererAffordances*) user;
        typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
        object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(sticky_foot_struc.object_name));
        KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;        
        drc::traj_opt_constraint_t trajmsg;
        trajmsg = get_desired_foot_motion_traj_constraint(self->last_state_msg_timestamp,self->robot_name,T_world_object,sticky_foot_struc,ee_name);   
        self->lcm->publish(channel, &trajmsg);
    }
    //----------------------------------------------------------------------------------------------------------
 
    static void publish_EE_locii_and_get_manip_plan (void *user, bool is_retractable)
    {
        RendererAffordances *self = (RendererAffordances*) user;

        typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
        object_instance_map_type_::iterator it = self->affCollection->_objects.find(self->object_selection);
    
        map<string, vector<KDL::Frame> > ee_frames_map;
        map<string, vector<int64_t> > ee_frame_timestamps_map;
        map<string, vector<double> > joint_pos_map;
        map<string, vector<int64_t> > joint_pos_timestamps_map;    
        
        
        //TODO: Replace with get_motion_constraints_via_batch_fk(std::vector<string> dof_names,std::vector<double> dof_current, std::vector<double> dof_des,is_retractable,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map)
        
        
        // The number of accumulated motion constraints for each seed may change slighlty due to distance threshold in accumulation, 
        // we can only use the associated ones.
        int max_motion_history_size = 0;
        int min_motion_history_size = 1000;
        self->stickyHandCollection->get_motion_history_bnds_of_seeds(it->first,max_motion_history_size,min_motion_history_size);
        self->stickyFootCollection->get_motion_history_bnds_of_seeds(it->first,max_motion_history_size,min_motion_history_size);
        
        int max_num_frames = min_motion_history_size;
        double retracting_offset = 0.1; //10cm  for sandia its in the x axis.  
         // Publish time indexed ee motion constraints from associated sticky hands 
        self->stickyHandCollection->get_motion_constraints(it->first,it->second,is_retractable,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map,max_num_frames,retracting_offset);
         // Publish time indexed ee motion constraints from associated sticky feet 
        self->stickyFootCollection->get_motion_constraints(it->first,it->second,is_retractable,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map,max_num_frames,retracting_offset);

        
        
        string channel  ="DESIRED_MANIP_PLAN_EE_LOCI"; 
     bool unique_ee_occurances=true;
     publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);
    } 
  
    //----------------------------------------------------------------------------------------------------   

  static void publish_pose_goal (void *user, string channel,KDL::Frame& T_world_body_desired,bool to_future_state)
  {
      RendererAffordances *self = (RendererAffordances*) user;

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->affCollection->_objects.find(self->object_selection);
  
      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
  
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map;    
  
  
     // Publish time indexed ee endpose constraints from associated sticky hands 
     self->stickyHandCollection->get_pose_constraints(it->first,it->second,to_future_state,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map);
     // Publish time indexed ee endpose constraints from associated sticky feet 
     self->stickyFootCollection->get_pose_constraints(it->first,it->second,to_future_state,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map);
  
      //body pose constraint  (current pelvis orientation may be considered during pose optimization as a loose constraint)
      vector<KDL::Frame> T_world_ee_frames;
      vector<int64_t> frame_timestamps;
      T_world_ee_frames.push_back(T_world_body_desired);
      frame_timestamps.push_back(0);               
      ee_frames_map.insert(make_pair("pelvis", T_world_ee_frames));
      ee_frame_timestamps_map.insert(make_pair("pelvis", frame_timestamps));      

     bool unique_ee_occurances=true;
     publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);
  } 
   //----------------------------------------------------------------------------------------------------   
  static void publish_pose_goal_4_aff_motion (void *user, string channel,KDL::Frame& T_world_body_desired)
  {
      
      RendererAffordances *self = (RendererAffordances*) user;

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->affCollection->_objects.find(self->object_selection);
  
      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
  
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map; 
  
      bool is_retractable = false;
      // The number of accumulated motion constraints for each seed may change slighlty due to hand frame distance threshold in accumulation, 
      // we can only use the associated ones.
      int max_motion_history_size = 0;
      int min_motion_history_size = 1000;
      self->stickyHandCollection->get_motion_history_bnds_of_seeds(it->first,max_motion_history_size,min_motion_history_size);
      self->stickyFootCollection->get_motion_history_bnds_of_seeds(it->first,max_motion_history_size,min_motion_history_size);
      
      int max_num_frames = min_motion_history_size;
       // Publish time indexed ee motion constraints from associated sticky hands 
      double retracting_offset = 0.1; //10cm  for sandia its in the x axis.  
      self->stickyHandCollection->get_motion_constraints(it->first,it->second,is_retractable,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map,max_num_frames,retracting_offset);
       // Publish time indexed ee motion constraints from associated sticky feet 
      self->stickyFootCollection->get_motion_constraints(it->first,it->second,is_retractable,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map,max_num_frames,retracting_offset);    
  
      //body pose constraint  (current pelvis orientation may be considered during pose optimization as a loose constraint)
     /* vector<KDL::Frame> T_world_ee_frames;
      vector<int64_t> frame_timestamps;
      T_world_ee_frames.push_back(T_world_body_desired);
      frame_timestamps.push_back(0);               
      ee_frames_map.insert(make_pair("pelvis", T_world_ee_frames));
      ee_frame_timestamps_map.insert(make_pair("pelvis", frame_timestamps));   */   

     bool unique_ee_occurances=true;
     publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);  
  }

  
  //----------------------------------------------------------------------------------------------------   
  
  static void publish_EE_goal_sequence_and_get_whole_body_plan (void *user, string channel, bool to_future_state, bool supress_hands,bool use_nan_for_z)
  {
      RendererAffordances *self = (RendererAffordances*) user;

  
      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
  
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map;   
  
    bool is_time_ordered = true;
      if(!supress_hands)
      {
        // get time indexed ee goal constraints from selected sticky hands 
        self->stickyHandCollection->get_selected_pose_constraints(self->affCollection,to_future_state,
                                                                   self->seedSelectionManager,
                                                                   ee_frames_map,ee_frame_timestamps_map,
                                                                   joint_pos_map,joint_pos_timestamps_map,
                                                                   is_time_ordered);
      }
      
     // Publish time indexed ee goal constraints from selected sticky feet 
     self->stickyFootCollection->get_selected_pose_constraints(self->affCollection,to_future_state,
                                                                   self->seedSelectionManager,
                                                                   ee_frames_map,ee_frame_timestamps_map,
                                                                   joint_pos_map,joint_pos_timestamps_map,
                                                                   is_time_ordered,use_nan_for_z);
                                                                   
     // shift selection append a selection order id to ee names in get_time_ordered_pose_constraints                                                               
     // setting unique_ee_occurances to false, accounts for that          
     bool unique_ee_occurances=false; 
     publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);
  } 
  
  //----------------------------------------------------------------------------------------------------   
  static void publish_EE_goal_sequence_as_poal_goal (void *user, string channel, bool to_future_state, bool supress_hands,bool use_nan_for_z)
  {
      RendererAffordances *self = (RendererAffordances*) user;

  
      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
  
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map;   
  
      bool is_time_ordered = false;
      if(!supress_hands)
      {
       // get time indexed ee goal constraints from selected sticky hands 
       self->stickyHandCollection->get_selected_pose_constraints(self->affCollection,to_future_state,
                                                                   self->seedSelectionManager,
                                                                   ee_frames_map,ee_frame_timestamps_map,
                                                                   joint_pos_map,joint_pos_timestamps_map,
                                                                   is_time_ordered);
      }
      
     // Publish time indexed ee goal constraints from selected sticky feet 
     self->stickyFootCollection->get_selected_pose_constraints(self->affCollection,to_future_state,
                                                                   self->seedSelectionManager,
                                                                   ee_frames_map,ee_frame_timestamps_map,
                                                                   joint_pos_map,joint_pos_timestamps_map,
                                                                   is_time_ordered,use_nan_for_z);
     // shift selection append a selection order id to ee names in get_time_ordered_pose_constraints                                                               
     // setting unique_ee_occurances to false, accounts for that          
     bool unique_ee_occurances=false; 
     publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);
  } 
  
  
  //----------------------------------------------------------------------------------------------------   
  // Publish time indexed ee motion constraints from the selected sticky hand for end pose search
  static void publish_pose_goal_to_sticky_hand (void *user,string channel, StickyHandStruc &handstruc, KDL::Frame& T_world_body_desired, bool to_future_state,bool end_state_only)
  {
      RendererAffordances *self = (RendererAffordances*) user;

      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map;  
     
      string host_name = handstruc.object_name;
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->affCollection->_objects.find(host_name);  
      
      // populate message cache
      get_endpose_search_constraints_from_sticky_hand(handstruc,it->second,T_world_body_desired,to_future_state,end_state_only,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map);
      // Publish the message 
     bool unique_ee_occurances=true;
     publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);
  } 
  
  //----------------------------------------------------------------------------------------------------   
  // Publish time indexed ee motion constraints from the selected sticky hand   
  static void publish_EE_locii_for_manip_plan_from_sticky_hand(void *user,string channel, StickyHandStruc &handstruc, bool is_retracting, bool is_cyclic)
  {
     RendererAffordances *self = (RendererAffordances*) user;

      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map;  
     
      string host_name = handstruc.object_name;
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->affCollection->_objects.find(host_name);  
          
      double retracting_offset = 0.1; //10cm  for sandia its in the x axis.  
      get_motion_constraints_from_sticky_hand(handstruc,it->second, 
                                              is_retracting,is_cyclic,
                                              ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map,retracting_offset);
                                              
      bool unique_ee_occurances=true;
      publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);
  
  }
  
  //----------------------------------------------------------------------------------------------------   
   // Publish time indexed ee motion constraints from the selected sticky foot
  static void publish_pose_goal_to_sticky_foot (void *user,string channel, StickyFootStruc &footstruc, KDL::Frame& T_world_body_desired, bool to_future_state,bool end_state_only)
  {
      RendererAffordances *self = (RendererAffordances*) user;

      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map;  
     
      string host_name = footstruc.object_name;
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->affCollection->_objects.find(host_name);  
      
      // populate message cache
      get_endpose_search_constraints_from_sticky_foot(footstruc,it->second,T_world_body_desired,to_future_state,end_state_only,ee_frames_map,ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map);
      // Publish the message 
     bool unique_ee_occurances=true;
     publish_traj_opt_constraint(channel,unique_ee_occurances,
                                 ee_frames_map,ee_frame_timestamps_map,
                                 joint_pos_map,joint_pos_timestamps_map,self);
  }   
    
 
  //----------------------------------------------------------------------------------------------------   
  static void publish_ee_transform_to_engage_ee_teleop(const string& channel,int ee_selection,KDL::Vector worldframe_mateaxis,KDL::Frame &T_aff_ee,void* user)
  {
     RendererAffordances *self = (RendererAffordances*) user;
     drc::ee_teleop_transform_t msg;
     drc::position_3d_t hand2aff_offset;
     double x,y,z,w;
     T_aff_ee.M.GetQuaternion(x,y,z,w);
     hand2aff_offset.translation.x = T_aff_ee.p[0];
     hand2aff_offset.translation.y = T_aff_ee.p[1];
     hand2aff_offset.translation.z = T_aff_ee.p[2];
     hand2aff_offset.rotation.x = x;
     hand2aff_offset.rotation.y = y;
     hand2aff_offset.rotation.z = z;
     hand2aff_offset.rotation.w = w; 
     msg.hand2aff_offset = hand2aff_offset;
     msg.mate_axis.x=worldframe_mateaxis[0]; 
     msg.mate_axis.y=worldframe_mateaxis[1];
     msg.mate_axis.z=worldframe_mateaxis[2];
     msg.ee_type=ee_selection;   
     self->lcm->publish(channel, &msg); 
  }  
  //----------------------------------------------------------------------------------------------------         

}// end namespace

#endif //RENDERER_AFFORDANCES_LCM_UTILS_HPP
