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
        for(map<string,vector<KDL::Frame> >::iterator it = ee_frames_map.begin(); it!=ee_frames_map.end(); it++) { 
            string ee_name = it->first;
            vector<KDL::Frame> ee_frames  = it->second;
            map<string, vector<drc::affordance_index_t> >::iterator ts_it = ee_frame_affindices_map.find(it->first);
            if(ts_it == ee_frame_affindices_map.end()){
                cerr << "ERROR: No Aff index found for ee " << it->first << endl;      
                return;
            }
 
            vector<drc::affordance_index_t> ee_frame_affindices = ts_it->second; 
            for(uint i = 0; i < (uint) ee_frames.size(); i++) {   
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

     static void publish_partial_grasp_state_for_execution( StickyHandStruc &sticky_hand_struc,string ee_name, string channel,KDL::Frame &T_world_geometry, 
                                                           int grasp_state, void *user)
    {
        //grasp state = 0 - open
        //grasp state = 1 - partial 
        //grasp state = 2 - full grasp 
        RendererAffordances *self = (RendererAffordances*) user;
        drc::desired_grasp_state_t msg;
        msg.utime = self->last_state_msg_timestamp;
        msg.robot_name = self->robot_name;
    
        msg.object_name = string(sticky_hand_struc.object_name);
        msg.geometry_name = string(sticky_hand_struc.geometry_name);
        msg.unique_id = sticky_hand_struc.uid;
        msg.grasp_type = sticky_hand_struc.hand_type;
        msg.power_grasp = 0;
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
                //if(grasp_flag){
                if(grasp_state == 2){
                    msg.l_joint_position[i]= sticky_hand_struc.joint_position[i];
                }
                else if(grasp_state == 1){
                    if(i>=9)
                        msg.l_joint_position[i]=0;
                    else  if(i %3 == 1){
                        msg.l_joint_position[i]= sticky_hand_struc.joint_position[i]; ///2.0;
                    }
                    else{
                        msg.l_joint_position[i]=0;
                    }
                }
                else if(grasp_state == 3){
                    if(i>=9)
                        msg.l_joint_position[i]=0;
                    else{
                        msg.l_joint_position[i]= sticky_hand_struc.joint_position[i]; ///2.0;
                    }
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
                if(grasp_state == 2){
                    msg.r_joint_position[i]= sticky_hand_struc.joint_position[i];
                }
                else if(grasp_state == 1){
                    if(i>=9)
                        msg.r_joint_position[i]=0;
                    else if(i %3 == 1){
                        msg.r_joint_position[i]= sticky_hand_struc.joint_position[i]; ///2.0;
                    }
                    else{
                        msg.r_joint_position[i]=0;
                    }
                }
                else if(grasp_state == 3){
                    if(i>=9)
                        msg.r_joint_position[i]=0;
                    else{
                        msg.r_joint_position[i]= sticky_hand_struc.joint_position[i]; ///2.0;
                    }
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
  
    // one traj_opt_constraint
    // for N ee's and K timesteps.    
  
    static void publish_traj_opt_constraint(const string& channel, map<string, vector<KDL::Frame> > &ee_frames_map,
                                            map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                            map<string, vector<double> > &joint_pos_map,
                                            map<string, vector<int64_t> > &joint_pos_timestamps_map,  void *user)
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
        int num_joints = 0;
        // N joints's and K keyframes
        for(map<string,vector<double> >::iterator it = joint_pos_map.begin(); it!=joint_pos_map.end(); it++)
            { 
                string joint_name = it->first;
                vector<double> joint_pos  = it->second;
                map<string, vector<int64_t> >::iterator ts_it = joint_pos_timestamps_map.find(it->first);
                if(ts_it == joint_pos_timestamps_map.end()){
                    cerr << "ERROR: No Timestamp found for joint " << it->first << endl;      
                    return;
                }
 
                vector<int64_t> joint_pos_timestamps = ts_it->second; 
                for(uint i = 0; i < (uint) joint_pos.size(); i++)
                    {   
                        trajmsg.joint_name.push_back(joint_name);
                        trajmsg.joint_position.push_back(joint_pos[i]);  
                        int64_t time_stamp = joint_pos_timestamps[i];
                        trajmsg.joint_timestamps.push_back(time_stamp);   
                        num_joints++;  
                    } // end for joint pos's
            } // end for joints's    
    
        trajmsg.num_joints =num_joints; 
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
        else
            {
                KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
                KDL::Vector handframe_offset;
                handframe_offset[0]=0.01;handframe_offset[1]=0;handframe_offset[2]=0;
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
    static void publish_grasp_state_for_execution( StickyHandStruc &sticky_hand_struc,string ee_name, string channel,KDL::Frame &T_world_geometry,  bool grasp_flag,bool power_flag, void *user)
    {
        RendererAffordances *self = (RendererAffordances*) user;
        drc::desired_grasp_state_t msg;
        msg.utime = self->last_state_msg_timestamp;
        msg.robot_name = self->robot_name;
    
        msg.object_name = string(sticky_hand_struc.object_name);
        msg.geometry_name = string(sticky_hand_struc.geometry_name);
        msg.unique_id = sticky_hand_struc.uid;
        msg.grasp_type = sticky_hand_struc.hand_type;
        msg.power_grasp = power_flag;
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
    static void publish_eegoal_to_sticky_foot(boost::shared_ptr<lcm::LCM> &_lcm, StickyFootStruc &sticky_foot_struc,
                                              string ee_name, string channel,KDL::Frame &T_world_geometry,bool reach_flag)
    {
        drc::ee_goal_t goalmsg;
        goalmsg.robot_name = "atlas";
        goalmsg.root_name = "pelvis";
        goalmsg.ee_name = ee_name;

        // desired ee position in world frame
        KDL::Frame T_world_ee,T_body_ee;
    
        KDL::Frame  T_geometry_foot = sticky_foot_struc.T_geometry_foot;  
        T_world_ee = T_world_geometry*T_geometry_foot;
             
        if(reach_flag) {
            // Set the reach position to be slightly off the sticky foot
            KDL::Vector footframe_offset(0.0, 0.0, 0.025);
            //footframe_offset[0]=0.0;footframe_offset[1]=0; footframe_offset[2]=-0.05;
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
 
    static void publish_EE_locii_and_get_manip_plan (void *user, bool is_retractable)
    {
        RendererAffordances *self = (RendererAffordances*) user;

        typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
        object_instance_map_type_::iterator it = self->instantiated_objects.find(self->object_selection);
    
        map<string, vector<KDL::Frame> > ee_frames_map;
        map<string, vector<int64_t> > ee_frame_timestamps_map;
    
        map<string, vector<double> > joint_pos_map;
        map<string, vector<int64_t> > joint_pos_timestamps_map;    
    
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
                        for(uint i = 0; i < (uint) num_frames; i++) {
                            KDL::Frame T_object_hand = hand_it->second._gl_hand->_desired_body_motion_history[i];
                            KDL::Frame T_world_hand = T_world_object*T_object_hand;
                            KDL::Frame T_world_ee = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame
                            T_world_ee_frames.push_back(T_world_ee);
                            int64_t timestamp=(int64_t)i*1000000;
                            frame_timestamps.push_back(timestamp);   
                        }
                        // append reverse motion with a small back up palm offset
                        if(is_retractable){
                            for(uint i = 0; i < (uint) num_frames; i++) {
                                KDL::Frame T_object_hand = hand_it->second._gl_hand->_desired_body_motion_history[num_frames-1-i];                   
                                KDL::Frame T_world_hand = T_world_object*T_object_hand;
                                KDL::Frame T_world_ee = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame                             
                                KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
                                KDL::Vector handframe_offset;
                                handframe_offset[0]=0.05;handframe_offset[1]=0;handframe_offset[2]=0;
                                KDL::Vector palmframe_offset= T_palm_hand*handframe_offset;
                                KDL::Vector worldframe_offset=T_world_ee.M*palmframe_offset;
                                T_world_ee.p += worldframe_offset;   
                                T_world_ee_frames.push_back(T_world_ee);
                                int64_t timestamp=(int64_t)(num_frames+i)*1000000;
                                frame_timestamps.push_back(timestamp);   
                            } 
                        }
                        //===================         

                        ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
                        ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));   
                        // hand_it->second.joint_name;  
                        // hand_it->second.joint_position; 
           
                          for(uint k = 0; k< (uint) hand_it->second.joint_name.size(); k++) 
                          {
                            std::string joint_name = hand_it->second.joint_name[k];
                            vector<double> joint_pos;
                            vector<int64_t> joint_pos_timestamps;
                            uint i=0;
                            //for(uint i = 0; i < (uint) num_frames; i++)
                            //{ 
                            joint_pos.push_back(hand_it->second.joint_position[k]);
                            int64_t timestamp=(int64_t)i*1000000;
                            joint_pos_timestamps.push_back(timestamp);
                            //}
                            // append reverse motion with a small back up palm offset
                            if(is_retractable){
                              //for(uint i = 0; i < (uint) num_frames; i++)
                                  //{
                                  //joint_pos.push_back(0.5*hand_it->second.joint_position[k]); % dont open fully, just loosen hold until half way
                                  joint_pos.push_back(0.0);
              
                                  int64_t timestamp=(int64_t)(num_frames+i)*1000000;
                                  joint_pos_timestamps.push_back(timestamp);   
                                  //} 
                              }
                              joint_pos_map.insert(make_pair(joint_name,joint_pos));
                              joint_pos_timestamps_map.insert(make_pair(joint_name, joint_pos_timestamps));  
                          }
           
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
        publish_traj_opt_constraint(channel, ee_frames_map, ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map, self);
    } 
  
//==================================

  static void publish_pose_goal (void *user, string channel,KDL::Frame& T_world_body_desired)
  {
      RendererAffordances *self = (RendererAffordances*) user;

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->instantiated_objects.find(self->object_selection);
  
      map<string, vector<KDL::Frame> > ee_frames_map;
      map<string, vector<int64_t> > ee_frame_timestamps_map;
  
      map<string, vector<double> > joint_pos_map;
      map<string, vector<int64_t> > joint_pos_timestamps_map;    
  
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
                  
                  KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.
                  if(!it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
                        cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;

                  int num_frames = 1;
                  vector<KDL::Frame> T_world_ee_frames;
                  vector<int64_t> frame_timestamps;
                  for(uint i = 0; i < (uint) num_frames; i++) {
                      KDL::Frame  T_world_ee = T_world_graspgeometry*T_geometry_palm;   
                      KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
                      KDL::Vector handframe_offset;
                      handframe_offset[0]=0.01;handframe_offset[1]=0;handframe_offset[2]=0;
                      KDL::Vector palmframe_offset= T_palm_hand*handframe_offset;
                      KDL::Vector worldframe_offset=T_world_ee.M*palmframe_offset;
                      T_world_ee.p += worldframe_offset;                    
                      
                      T_world_ee_frames.push_back(T_world_ee);
                      int64_t timestamp=(int64_t)i*1000000;
                      frame_timestamps.push_back(timestamp);   
                  }
                  //===================         

                  ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
                  ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));   
                  // hand_it->second.joint_name;  
                  // hand_it->second.joint_position; 
     
                    for(uint k = 0; k< (uint) hand_it->second.joint_name.size(); k++) 
                    {
                      std::string joint_name = hand_it->second.joint_name[k];
                      vector<double> joint_pos;
                      vector<int64_t> joint_pos_timestamps;
                      uint i=0;
                      joint_pos.push_back(hand_it->second.joint_position[k]);
                      int64_t timestamp=(int64_t)i*1000000;
                      joint_pos_timestamps.push_back(timestamp);
                      joint_pos_map.insert(make_pair(joint_name,joint_pos));
                      joint_pos_timestamps_map.insert(make_pair(joint_name, joint_pos_timestamps));  
                    }
     
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
                  KDL::Frame  T_geometry_foot = KDL::Frame::Identity(); 
                  if(!foot_it->second._gl_foot->get_link_frame(ee_name,T_geometry_foot))
                     cout <<"ERROR: ee link "<< ee_name << " not found in sticky foot urdf"<< endl;

                  KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
                  KDL::Frame  T_world_geometry = KDL::Frame::Identity(); 
                  if(!it->second._gl_object->get_link_geometry_frame(string(foot_it->second.geometry_name),T_world_geometry))
                        cerr << " failed to retrieve " << foot_it->second.geometry_name<<" in object " << foot_it->second.object_name <<endl;

                  int num_frames =  1;
                  vector<KDL::Frame> T_world_ee_frames;
                  vector<int64_t> frame_timestamps;
                  for(uint i = 0; i < (uint) num_frames; i++)
                      {
                          KDL::Frame  T_world_ee = T_world_geometry*T_geometry_foot;  
                          T_world_ee_frames.push_back(T_world_ee);
                          int64_t timestamp=(int64_t)i*1000000;
                          frame_timestamps.push_back(timestamp);   
                      }
                  //=================== 
      
                  ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
                  ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));    
              } // end if (host_name == (it->first))
      } // end for sticky feet
          
          
      //body pose constraint  (only orientation will be considered during pose optimization)
      vector<KDL::Frame> T_world_ee_frames;
      vector<int64_t> frame_timestamps;
      T_world_ee_frames.push_back(T_world_body_desired);
      frame_timestamps.push_back(0);               
      ee_frames_map.insert(make_pair("pelvis", T_world_ee_frames));
      ee_frame_timestamps_map.insert(make_pair("pelvis", frame_timestamps));      

      publish_traj_opt_constraint(channel, ee_frames_map, ee_frame_timestamps_map,joint_pos_map,joint_pos_timestamps_map,self);
  } 
    

  static void publish_mate_cmd (void *user, string channel)
  {
      RendererAffordances *self = (RendererAffordances*) user;

        /*drc::hongkai_is_awesome_t msg.;
        msg..utime = self->last_state_msg_timestamp;
        msg.robot_name = self->robot_name;*/

        //self->lcm->publish(channel, &msg.);

  } 
      

}// end namespace

#endif //RENDERER_AFFORDANCES_LCM_UTILS_HPP
