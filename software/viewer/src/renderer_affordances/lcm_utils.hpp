#ifndef RENDERER_AFFORDANCES_LCM_UTILS_HPP
#define RENDERER_AFFORDANCES_LCM_UTILS_HPP
#include "renderer_affordances.hpp"

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
    trajmsg.utime = self->last_state_msg_timestamp;
    trajmsg.robot_name = self->robot_name;
    
    int num_links = 0;
 
 // N ee's and K keyframes
    for(map<string,vector<KDL::Frame> >::iterator it = ee_frames_map.begin(); it!=ee_frames_map.end(); it++)
    { 
       string ee_name = it->first;
       vector<KDL::Frame> ee_frames  = it->second;
       map<string, vector<drc::affordance_index_t> >::iterator ts_it = ee_frame_affindices_map.find(it->first);
       if(ts_it == ee_frame_affindices_map.end()){
         cerr << "ERROR: No Aff index found for ee " << it->first << endl;      
         return;
        }
 
       vector<drc::affordance_index_t> ee_frame_affindices = ts_it->second; 
      for(uint i = 0; i < (uint) ee_frames.size(); i++)
      {   
           KDL::Frame T_world_ee = ee_frames[i];
           double x,y,z,w;
           T_world_ee.M.GetQuaternion(x,y,z,w);
           drc::position_3d_t pose;
	         pose.translation.x = T_world_ee.p[0];
	         pose.translation.y = T_world_ee.p[1];
	         pose.translation.z = T_world_ee.p[2];
           pose.rotation.x = x;
           pose.rotation.y = y;
           pose.rotation.z = z;
           pose.rotation.w = w; 
           trajmsg.link_name.push_back(ee_name);
           trajmsg.link_origin_position.push_back(pose);  
           trajmsg.link_aff_index.push_back(ee_frame_affindices[i]);   
           num_links++;  
      } // end for frames
    } // end for ee's
   trajmsg.num_links =  num_links;   
	 trajmsg.num_joints =0; 
   self->lcm->publish(channel, &trajmsg);
  }
  
  // one traj_opt_constraint
  // for N ee's and K timesteps.    
  static void publish_traj_opt_constraint(const string& channel, map<string, vector<KDL::Frame> > &ee_frames_map, map<string, vector<int64_t> > &ee_frame_timestamps_map,  void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    drc::traj_opt_constraint_t trajmsg;
    trajmsg.utime = self->last_state_msg_timestamp;
    trajmsg.robot_name = self->robot_name;
    
    int num_links = 0;
 
 // N ee's and K keyframes
    for(map<string,vector<KDL::Frame> >::iterator it = ee_frames_map.begin(); it!=ee_frames_map.end(); it++)
    { 
       string ee_name = it->first;
       vector<KDL::Frame> ee_frames  = it->second;
       map<string, vector<int64_t> >::iterator ts_it = ee_frame_timestamps_map.find(it->first);
       if(ts_it == ee_frame_timestamps_map.end()){
         cerr << "ERROR: No Timestamp found for ee " << it->first << endl;      
         return;
        }
 
       vector<int64_t> ee_frame_timestamps = ts_it->second; 
      for(uint i = 0; i < (uint) ee_frames.size(); i++)
      {   
           KDL::Frame T_world_ee = ee_frames[i];
           double x,y,z,w;
           T_world_ee.M.GetQuaternion(x,y,z,w);
           drc::position_3d_t pose;
	         pose.translation.x = T_world_ee.p[0];
	         pose.translation.y = T_world_ee.p[1];
	         pose.translation.z = T_world_ee.p[2];
           pose.rotation.x = x;
           pose.rotation.y = y;
           pose.rotation.z = z;
           pose.rotation.w = w; 
           trajmsg.link_name.push_back(ee_name);
           trajmsg.link_origin_position.push_back(pose);  
           int64_t time_stamp = ee_frame_timestamps[i];
           trajmsg.link_timestamps.push_back(time_stamp);   
           num_links++;  
      } // end for frames
    } // end for ee's
   trajmsg.num_links =  num_links;   
	 trajmsg.num_joints =0; 
   self->lcm->publish(channel, &trajmsg);
  }
 //----------------------------------------------------------------------------------------------------
  static void publish_eegoal_to_sticky_hand(boost::shared_ptr<lcm::LCM> &_lcm, StickyHandStruc &sticky_hand_struc,string ee_name, string channel,KDL::Frame &T_world_geometry,bool reach_flag)
  {
    drc::ee_goal_t goalmsg;
    goalmsg.robot_name = "atlas";
    goalmsg.root_name = "pelvis";
    goalmsg.ee_name = ee_name;

    // desired ee position in world frame
    KDL::Frame T_world_ee,T_body_ee;
    
// Must account for the mismatch between l_hand and base in sandia_hand urdf. Publish in palm frame.   
   KDL::Frame  T_geometry_hand = sticky_hand_struc.T_geometry_hand;  // this is actually in a base frame that is not l_hand/r_hand.
//    KDL::Frame T_base_palm = KDL::Frame::Identity();
//    // this was there in urdf to make sure fingers are pointing in z axis.
//    T_base_palm.M =  KDL::Rotation::RPY(0,-(M_PI/2),0); 
//    KDL::Frame  T_geometry_palm = T_geometry_base*T_base_palm.Inverse()

    KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
    if(!sticky_hand_struc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
      cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
      
    T_world_ee = T_world_geometry*T_geometry_palm;
             
   if(reach_flag)
   {
    KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
    KDL::Vector handframe_offset;
    handframe_offset[0]=0.1;handframe_offset[1]=0;handframe_offset[2]=0;
    KDL::Vector palmframe_offset= T_palm_hand*handframe_offset;
    KDL::Vector worldframe_offset=T_world_ee.M*palmframe_offset;
    T_world_ee.p += worldframe_offset;

   }  

    //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.

    // desired ee position wrt to robot body.
    //T_body_ee = T_body_world*T_world_ee;
    T_body_ee = T_world_ee; // send them in world frame for now.
    double x,y,z,w;
    T_body_ee.M.GetQuaternion(x,y,z,w);

    goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
    goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
    goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

    goalmsg.ee_goal_pos.rotation.x = x;
    goalmsg.ee_goal_pos.rotation.y = y;
    goalmsg.ee_goal_pos.rotation.z = z;
    goalmsg.ee_goal_pos.rotation.w = w;

    goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

    goalmsg.num_chain_joints  = sticky_hand_struc.joint_name.size();
    // No specified posture bias
    goalmsg.use_posture_bias  = false;
    goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
    goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
    for(int i = 0; i < goalmsg.num_chain_joints; i++){
      if(!reach_flag){
        goalmsg.joint_posture_bias[i]=sticky_hand_struc.joint_position[i];
      }
      else{
        goalmsg.joint_posture_bias[i]=0;//sticky_hand_struc.joint_position[i];
      }
      goalmsg.chain_joint_names[i]= sticky_hand_struc.joint_name[i];
    }

    // Publish the message
    goalmsg.halt_ee_controller = false;

    _lcm->publish(channel, &goalmsg);
  }  
 //----------------------------------------------------------------------------------------------------   
  static void publish_desired_hand_motion(StickyHandStruc &sticky_hand_struc, string ee_name, string channel,  void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    drc::traj_opt_constraint_t trajmsg;
    trajmsg.utime = self->last_state_msg_timestamp;
    trajmsg.robot_name = self->robot_name;
    
    KDL::Frame  T_geometry_hand = sticky_hand_struc.T_geometry_hand;
    KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
    if(!sticky_hand_struc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
    cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
    KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset


    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(sticky_hand_struc.object_name));
    KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;

    trajmsg.num_links =  sticky_hand_struc._gl_hand->_desired_body_motion_history.size();
    for(uint i = 0; i < (uint) trajmsg.num_links; i++)
    {
       double x,y,z,w;
       KDL::Frame T_object_hand = sticky_hand_struc._gl_hand->_desired_body_motion_history[i];
       KDL::Frame T_world_hand = T_world_object*T_object_hand;
       KDL::Frame nextTfframe = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame
       nextTfframe.M.GetQuaternion(x,y,z,w);

       drc::position_3d_t pose;
       pose.translation.x = nextTfframe.p[0];
       pose.translation.y = nextTfframe.p[1];
       pose.translation.z = nextTfframe.p[2];
       pose.rotation.x = x;
       pose.rotation.y = y;
       pose.rotation.z = z;
       pose.rotation.w = w; 
       trajmsg.link_name.push_back(ee_name);
       trajmsg.link_origin_position.push_back(pose);  
       trajmsg.link_timestamps.push_back(i);     
    }

    trajmsg.num_joints =0;
     self->lcm->publish(channel, &trajmsg);
  }
  
 //----------------------------------------------------------------------------------------------------
  static void publish_grasp_state_for_execution( StickyHandStruc &sticky_hand_struc,string ee_name, string channel,KDL::Frame &T_world_geometry,  bool grasp_flag, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    drc::desired_grasp_state_t msg;
    msg.utime = self->last_state_msg_timestamp;
    msg.robot_name = self->robot_name;
    
    msg.object_name = string(sticky_hand_struc.object_name);
    msg.geometry_name = string(sticky_hand_struc.geometry_name);
    msg.unique_id = sticky_hand_struc.uid;
    msg.grasp_type = sticky_hand_struc.hand_type;

    // desired ee position in world frame
    KDL::Frame T_world_ee,T_body_ee;
    
// Must account for the mismatch between l_hand and base in sandia_hand urdf. Publish in palm frame.   
   KDL::Frame  T_geometry_hand = sticky_hand_struc.T_geometry_hand;
//    KDL::Frame T_hand_palm = KDL::Frame::Identity();
//    // this was there in urdf to make sure fingers are pointing in z axis.
//    T_hand_palm.M =  KDL::Rotation::RPY(0,-(M_PI/2),0); 
//    KDL::Frame  T_geometry_palm = T_geometry_hand*T_hand_palm.Inverse()

    KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
    if(!sticky_hand_struc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
      cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;


    
    T_world_ee = T_world_geometry*T_geometry_palm;
    KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; // offset
    
    
    KDL::Frame T_hand_offset = KDL::Frame::Identity();
    
    // The palm frame is pointing in negative x axis. This is a convention for sticky hands.
    KDL::Frame T_palm_offset =  T_palm_hand*T_hand_offset;
    
    // cout <<"before offset"<<T_world_ee.p[0]<<" "<<T_world_ee.p[1]<<" "<<T_world_ee.p[2] << endl;
    T_world_ee = T_world_geometry*T_geometry_palm*T_palm_offset;
    //cout <<"after offset"<<T_world_ee.p[0]<<" "<<T_world_ee.p[1]<<" "<<T_world_ee.p[2] << endl;

     
    // double ro,pi,ya;
    //   T_world_ee.M.GetRPY(ro,pi,ya);
    //   cout <<"roll"<<ro*(180/M_PI) << endl;
    //   cout <<"pitch"<<pi*(180/M_PI) << endl;
    //   cout <<"yaw"<<ya*(180/M_PI) << endl;
       
          
    //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.

    // desired ee position wrt to robot body.
    //T_body_ee = T_body_world*T_world_ee;
    T_body_ee = T_world_ee; // send them in world frame for now.
    double x,y,z,w;
    T_body_ee.M.GetQuaternion(x,y,z,w);
    
    drc::position_3d_t hand_pose; 

    hand_pose.translation.x = T_body_ee.p[0];
    hand_pose.translation.y = T_body_ee.p[1];
    hand_pose.translation.z = T_body_ee.p[2];

    hand_pose.rotation.x = x;
    hand_pose.rotation.y = y;
    hand_pose.rotation.z = z;
    hand_pose.rotation.w = w;
    
    if((msg.grasp_type == msg.SANDIA_LEFT)||(msg.grasp_type == msg.IROBOT_LEFT)){
      msg.l_hand_pose = hand_pose;
      msg.num_l_joints  = sticky_hand_struc.joint_name.size();
      msg.num_r_joints  = 0;
      msg.l_joint_name.resize(msg.num_l_joints);
      msg.l_joint_position.resize(msg.num_l_joints);
      for(int i = 0; i < msg.num_l_joints; i++){
        if(grasp_flag){
          msg.l_joint_position[i]=sticky_hand_struc.joint_position[i];
        }
        else{
          msg.l_joint_position[i]=0;//sticky_hand_struc.joint_position[i];
        }
        msg.l_joint_name[i]= sticky_hand_struc.joint_name[i];
      }
    }
    else if((msg.grasp_type == msg.SANDIA_RIGHT)||(msg.grasp_type == msg.IROBOT_RIGHT)){
      msg.r_hand_pose = hand_pose;
      msg.num_r_joints  = sticky_hand_struc.joint_name.size();
      msg.num_l_joints  = 0;
      msg.r_joint_name.resize(msg.num_r_joints);
      msg.r_joint_position.resize(msg.num_r_joints);
      for(int i = 0; i < msg.num_r_joints; i++){
        if(grasp_flag){
          msg.r_joint_position[i]=sticky_hand_struc.joint_position[i];
        }
        else{
          msg.r_joint_position[i]=0;//sticky_hand_struc.joint_position[i];
        }
        msg.r_joint_name[i]= sticky_hand_struc.joint_name[i];
      }
    }

    // Publish the message 
    self->lcm->publish(channel, &msg);
  }
  
  //----------------------------------------------------------------------------------------------------
  static void publish_eegoal_to_sticky_foot(boost::shared_ptr<lcm::LCM> &_lcm, StickyFootStruc &sticky_foot_struc,string ee_name, string channel,KDL::Frame &T_world_geometry,bool reach_flag)
  {
    drc::ee_goal_t goalmsg;
    goalmsg.robot_name = "atlas";
    goalmsg.root_name = "pelvis";
    goalmsg.ee_name = ee_name;

    // desired ee position in world frame
    KDL::Frame T_world_ee,T_body_ee;
    
   KDL::Frame  T_geometry_foot = sticky_foot_struc.T_geometry_foot;  
   T_world_ee = T_world_geometry*T_geometry_foot;
             
   if(reach_flag)
   {
    KDL::Vector footframe_offset;
    footframe_offset[0]=0.0;footframe_offset[1]=0;footframe_offset[2]=-0.05;
    KDL::Vector worldframe_offset=T_world_ee.M*footframe_offset;
    T_world_ee.p += worldframe_offset;
   }  

    //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.

    // desired ee position wrt to robot body.
    //T_body_ee = T_body_world*T_world_ee;
    T_body_ee = T_world_ee; // send them in world frame for now.
    double x,y,z,w;
    T_body_ee.M.GetQuaternion(x,y,z,w);

    goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
    goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
    goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

    goalmsg.ee_goal_pos.rotation.x = x;
    goalmsg.ee_goal_pos.rotation.y = y;
    goalmsg.ee_goal_pos.rotation.z = z;
    goalmsg.ee_goal_pos.rotation.w = w;

    goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

    goalmsg.num_chain_joints  = sticky_foot_struc.joint_name.size();
    // No specified posture bias
    goalmsg.use_posture_bias  = false;
    goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
    goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
    for(int i = 0; i < goalmsg.num_chain_joints; i++){
      if(!reach_flag){
        goalmsg.joint_posture_bias[i]=sticky_foot_struc.joint_position[i];
      }
      else{
        goalmsg.joint_posture_bias[i]=0;//sticky_foot_struc.joint_position[i];
      }
      goalmsg.chain_joint_names[i]= sticky_foot_struc.joint_name[i];
    }

    // Publish the message
    goalmsg.halt_ee_controller = false;

    _lcm->publish(channel, &goalmsg);
  }  
 //----------------------------------------------------------------------------------------------------   
  static void publish_desired_foot_motion(StickyFootStruc &sticky_foot_struc, string ee_name, string channel,  void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    drc::traj_opt_constraint_t trajmsg;
    trajmsg.utime = self->last_state_msg_timestamp;
    trajmsg.robot_name = self->robot_name;
 
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(sticky_foot_struc.object_name));
    KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;
    
      trajmsg.num_links =  sticky_foot_struc._gl_foot->_desired_body_motion_history.size();
        for(uint i = 0; i < (uint) trajmsg.num_links; i++)
        {
           double x,y,z,w;
           KDL::Frame T_object_foot = sticky_foot_struc._gl_foot->_desired_body_motion_history[i];
           KDL::Frame T_world_foot = T_world_object*T_object_foot;
           KDL::Frame nextTfframe = T_world_foot;//TODO: Eventually will be in object frame
           nextTfframe.M.GetQuaternion(x,y,z,w);

           drc::position_3d_t pose;
	         pose.translation.x = nextTfframe.p[0];
	         pose.translation.y = nextTfframe.p[1];
	         pose.translation.z = nextTfframe.p[2];
           pose.rotation.x = x;
           pose.rotation.y = y;
           pose.rotation.z = z;
           pose.rotation.w = w; 
           trajmsg.link_name.push_back(ee_name);
           trajmsg.link_origin_position.push_back(pose);  
           trajmsg.link_timestamps.push_back(i);     
	      }
	      
	      trajmsg.num_joints =0;
     self->lcm->publish(channel, &trajmsg);
  }
 //----------------------------------------------------------------------------------------------------------
 
 static void get_manip_plan (void *user)
 {
    RendererAffordances *self = (RendererAffordances*) user;

    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(self->object_selection);
    
    map<string, vector<KDL::Frame> > ee_frames_map;
    map<string, vector<int64_t> > ee_frame_timestamps_map;
    // Publish time indexed ee motion constraints from associated sticky hands 
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      for(sticky_hands_map_type_::const_iterator hand_it = self->sticky_hands.begin(); hand_it!=self->sticky_hands.end(); hand_it++)
      {
         string host_name = hand_it->second.object_name;
         if (host_name == (it->first))
         {
            string ee_name;
            if(hand_it->second.hand_type==0)
              ee_name ="left_palm";    
            else if(hand_it->second.hand_type==1)   
               ee_name ="right_palm";    
            else
               cout << "unknown hand_type in on_otdf_dof_range_widget_popup_close\n";   
               
            // if ee_name already exists in ee_frames_map, redundant ee_frames
            // e.g two right sticky hands on the same object.
            map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
            if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
              return;   
            }
           
            //======================     
            KDL::Frame  T_geometry_hand = hand_it->second.T_geometry_hand;
            KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
            if(!hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_palm))
            cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
            KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset

            KDL::Frame T_world_object = it->second._gl_object->_T_world_body;

            int num_frames =  hand_it->second._gl_hand->_desired_body_motion_history.size();
            vector<KDL::Frame> T_world_ee_frames;
            vector<int64_t> frame_timestamps;
            for(uint i = 0; i < (uint) num_frames; i++)
            {
              KDL::Frame T_object_hand = hand_it->second._gl_hand->_desired_body_motion_history[i];
              KDL::Frame T_world_hand = T_world_object*T_object_hand;
              KDL::Frame T_world_ee = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame
              T_world_ee_frames.push_back(T_world_ee);
              int64_t timestamp=(int64_t)i*1000000;
              frame_timestamps.push_back(timestamp);   
            }
           //===================         

           ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
           ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));   
         } // end if (host_name == (it->first))
      } // end for sticky hands
        
     // Publish time indexed ee motion constraints from associated sticky feet 
     typedef map<string, StickyFootStruc > sticky_feet_map_type_;
      for(sticky_feet_map_type_::const_iterator foot_it = self->sticky_feet.begin(); foot_it!=self->sticky_feet.end(); foot_it++)
      {
         string host_name = foot_it->second.object_name;
         if (host_name == (it->first))
         {
         
            string ee_name;
            if(foot_it->second.foot_type==0)
               ee_name ="l_foot";    
            else if(foot_it->second.foot_type==1)   
               ee_name ="r_foot";    
            else
               cout << "unknown foot_type in on_otdf_dof_range_widget_popup_close\n";  
               
            // if ee_name already exists in ee_frames_map, redundant ee_frames
            // e.g two right sticky hands on the same object.
            map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
            if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
              return;   
            }      
          
            //======================     
            //KDL::Frame  T_geometry_ee = KDL::Frame::Identity(); 
            //if(!foot_it->second._gl_foot->get_link_frame(ee_name,T_geometry_ee))
            //    cout <<"ERROR: ee link "<< ee_name << " not found in sticky foot urdf"<< endl;

            KDL::Frame T_world_object = it->second._gl_object->_T_world_body;

            int num_frames =  foot_it->second._gl_foot->_desired_body_motion_history.size();
            vector<KDL::Frame> T_world_ee_frames;
            vector<int64_t> frame_timestamps;
            for(uint i = 0; i < (uint) num_frames; i++)
            {
              KDL::Frame T_object_foot = foot_it->second._gl_foot->_desired_body_motion_history[i];
              KDL::Frame T_world_foot = T_world_object*T_object_foot;
              KDL::Frame T_world_ee = T_world_foot;//TODO: Eventually will be in object frame
              T_world_ee_frames.push_back(T_world_ee);
              int64_t timestamp=(int64_t)i*1000000;
              frame_timestamps.push_back(timestamp);   
            }
           //=================== 
            
           ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
           ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));    
         } // end if (host_name == (it->first))
      } // end for sticky feet
      
   string channel  ="DESIRED_MANIP_PLAN_EE_LOCI"; 
   publish_traj_opt_constraint(channel, ee_frames_map, ee_frame_timestamps_map, self);
} 
  

}// end namespace

#endif //RENDERER_AFFORDANCES_LCM_UTILS_HPP
