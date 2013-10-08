#ifndef STICKYHAND_LCM_UTILS_HPP
#define STICKYHAND_LCM_UTILS_HPP
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace std;
using namespace boost;
//using namespace visualization_utils;

namespace visualization_utils
{

    inline static drc::ee_goal_t get_eegoal_to_sticky_hand(StickyHandStruc &sticky_hand_struc, string robot_name,string root_name,string ee_name,KDL::Frame &T_world_geometry,bool reach_flag)
    {
        drc::ee_goal_t goalmsg;
        goalmsg.robot_name = robot_name;
        goalmsg.root_name = root_name;
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

        goalmsg.halt_ee_controller = false;
        return goalmsg;
    }  

  //----------------------------------------------------------------------------------------------------------          
      
  inline static drc::traj_opt_constraint_t get_desired_hand_motion_traj_constraint(int64_t last_state_msg_timestamp, string robot_name,
                                                                                KDL::Frame &T_world_object, StickyHandStruc &sticky_hand_struc, string ee_name)
  {       
      drc::traj_opt_constraint_t trajmsg;
      trajmsg.utime = last_state_msg_timestamp;
      trajmsg.robot_name = robot_name;
  
      KDL::Frame  T_geometry_hand = sticky_hand_struc.T_geometry_hand;
      KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
      if(!sticky_hand_struc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
          cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
      KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset


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
      return trajmsg;
  } 
  //----------------------------------------------------------------------------------------------------------          
  inline static drc::desired_grasp_state_t get_grasp_state(int64_t last_state_msg_timestamp, string robot_name,
                                              StickyHandStruc &sticky_hand_struc,string ee_name,
                                              KDL::Frame &T_world_geometry,  bool grasp_flag,bool power_flag,bool squeeze_flag, double squeeze_amount)
  {
    drc::desired_grasp_state_t msg;
    msg.utime = last_state_msg_timestamp;
    msg.robot_name = robot_name;

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
              double gain = 1.0;
              if(squeeze_flag)
               gain= squeeze_amount;
              msg.l_joint_position[i]=gain*sticky_hand_struc.joint_position[i];
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
            double gain = 1.0;
            if(squeeze_flag)
              gain= squeeze_amount;
                msg.r_joint_position[i]=gain*sticky_hand_struc.joint_position[i];
            }
            else{
                msg.r_joint_position[i]=0;//sticky_hand_struc.joint_position[i];
            }
            msg.r_joint_name[i]= sticky_hand_struc.joint_name[i];
        }
    }
    
    msg.optimization_status = 0;
    
    return msg;
  } 
  //----------------------------------------------------------------------------------------------------------    

  inline static drc::desired_grasp_state_t  get_partial_grasp_state(int64_t last_state_msg_timestamp, string robot_name,
                                                              StickyHandStruc &sticky_hand_struc,string ee_name, 
                                                              KDL::Frame &T_world_geometry,int grasp_state)
  {
    drc::desired_grasp_state_t msg;
    msg.utime = last_state_msg_timestamp;
    msg.robot_name = robot_name;

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
                else  if(i%3 == 1){
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
                else if(i%3 == 1){
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
    }// end if else
    
     return msg;

  }  // end get_partial_grasp_state   

//-------------------------------------------------------------------------------------------

inline static void get_endpose_search_constraints_from_sticky_hand(StickyHandStruc &handstruc,OtdfInstanceStruc& obj, KDL::Frame& T_world_body_desired, bool to_future_state,
                                                  bool end_state_only,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map)
{
      string ee_name;
      if(handstruc.hand_type==drc::desired_grasp_state_t::SANDIA_LEFT)
          ee_name ="left_palm";    
      else if(handstruc.hand_type==drc::desired_grasp_state_t::SANDIA_RIGHT)   
          ee_name ="right_palm";    
      else if(handstruc.hand_type==drc::desired_grasp_state_t::IROBOT_LEFT)   
          ee_name ="left_base_link";
      else if(handstruc.hand_type==drc::desired_grasp_state_t::IROBOT_RIGHT)   
          ee_name ="right_base_link";        
      else
          cout << "unknown hand_type in get_endpose_search_constraints_from_sticky_hand\n";   

      // if ee_name already exists in ee_frames_map, redundant ee_frames
      // e.g two right sticky hands on the same object.
      map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
      if(ee_it!=ee_frames_map.end()){
          cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
          return;   
      }

      //======================     
      KDL::Frame  T_geometry_hand = handstruc.T_geometry_hand;
      KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
      if(!handstruc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
          cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
      KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset

      KDL::Frame T_world_object = KDL::Frame::Identity();
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();
      if(!to_future_state){
        T_world_object = obj._gl_object->_T_world_body;
        // the object might have moved.
        if(!obj._gl_object->get_link_geometry_frame(string(handstruc.geometry_name),T_world_graspgeometry))
            cerr << " failed to retrieve " << handstruc.geometry_name<<" in object " << handstruc.object_name <<endl;
        }
        else {
          T_world_object = obj._gl_object->_T_world_body_future;
          // the object might have moved.
          if(!obj._gl_object->get_link_geometry_future_frame(string(handstruc.geometry_name),T_world_graspgeometry))
            cerr << " failed to retrieve " << handstruc.geometry_name<<" in object " << handstruc.object_name <<endl;        
      }

      int num_frames = 1;
      if((to_future_state)&&(!end_state_only))
        num_frames = handstruc._gl_hand->_desired_body_motion_history.size();
      
      vector<KDL::Frame> T_world_ee_frames;
      vector<int64_t> frame_timestamps;
      for(uint i = 0; i < (uint) num_frames; i++) {
      
         KDL::Frame T_world_ee;      
          if((to_future_state)&&(!end_state_only)){
             KDL::Frame T_object_hand = handstruc._gl_hand->_desired_body_motion_history[i];
            //obj._gl_object->_T_world_body is the frame in which desired body is accumulated.
             KDL::Frame T_world_hand = obj._gl_object->_T_world_body*T_object_hand;
             T_world_ee = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame     
          }
          else
            T_world_ee = T_world_graspgeometry*T_geometry_palm;
              
          KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
          KDL::Vector handframe_offset;
          handframe_offset[0]=0.0;handframe_offset[1]=0;handframe_offset[2]=0;
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
      // handstruc.joint_name;  
      // handstruc.joint_position; 
      // posture constraints for transitions only
      
        for(uint k = 0; k< (uint) handstruc.joint_name.size(); k++) 
        {
          std::string joint_name = handstruc.joint_name[k];
          vector<double> joint_pos;
          vector<int64_t> joint_pos_timestamps;
          uint i=0;
          joint_pos.push_back(handstruc.joint_position[k]);
          int64_t timestamp=(int64_t)i*1000000;
          joint_pos_timestamps.push_back(timestamp);
          joint_pos_map.insert(make_pair(joint_name,joint_pos));
          joint_pos_timestamps_map.insert(make_pair(joint_name, joint_pos_timestamps));  
        }
          
      //body pose constraint  (only orientation will be considered during pose optimization)
      vector<KDL::Frame> T_world_ee_frames_root;
      vector<int64_t> frame_timestamps_root;
      for(uint i = 0; i < (uint) num_frames; i++) {
        T_world_ee_frames_root.push_back(T_world_body_desired);
        int64_t timestamp=(int64_t)i*1000000;
        frame_timestamps_root.push_back(timestamp);
      }
      ee_frames_map.insert(make_pair("pelvis", T_world_ee_frames_root));
      ee_frame_timestamps_map.insert(make_pair("pelvis", frame_timestamps_root)); 
           
} // end get_endpose_search_constraints_from_sticky_hand  
//----------------------------------------------------------------------------------------------------------    
           
}// end namespace

#endif //STICKYHAND_LCM_UTILS_HPP
