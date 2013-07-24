#ifndef STICKYFOOT_LCM_UTILS_HPP
#define STICKYFOOT_LCM_UTILS_HPP
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace std;
using namespace boost;
//using namespace visualization_utils;


namespace visualization_utils
{

    inline static drc::ee_goal_t get_eegoal_to_stickyfoot(StickyFootStruc &sticky_foot_struc, string robot_name,string root_name, string ee_name, KDL::Frame &T_world_geometry,bool reach_flag)
    {
        drc::ee_goal_t goalmsg;
        goalmsg.robot_name = robot_name;
        goalmsg.root_name = root_name;
        goalmsg.ee_name = ee_name;

        // desired ee position in world frame
        KDL::Frame T_world_ee,T_body_ee;
    
        KDL::Frame  T_geometry_foot = sticky_foot_struc.T_geometry_foot;  
        T_world_ee = T_world_geometry*T_geometry_foot;
             
        if(reach_flag) {
            // Set the reach position to be slightly off the sticky foot
            KDL::Vector footframe_offset(0.0, 0.0, 0.05);
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
        for(int i = 0; i < goalmsg.num_chain_joints; i++)
        {
          if(!reach_flag){
              goalmsg.joint_posture_bias[i]=sticky_foot_struc.joint_position[i];
          }
          else{
              goalmsg.joint_posture_bias[i]=0;//sticky_foot_struc.joint_position[i];
          }
          goalmsg.chain_joint_names[i]= sticky_foot_struc.joint_name[i];
        }

        goalmsg.halt_ee_controller = false;
        return goalmsg;
    } 
    
    //----------------------------------------------------------------------------------------------------------    
   inline static drc::traj_opt_constraint_t get_desired_foot_motion_traj_constraint(int64_t last_state_msg_timestamp, string robot_name,
                                                                                  KDL::Frame &T_world_object, StickyFootStruc &sticky_foot_struc, string ee_name)
    {
      drc::traj_opt_constraint_t trajmsg;
      trajmsg.utime = last_state_msg_timestamp;
      trajmsg.robot_name = robot_name;

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
      return trajmsg;
    }      
    //----------------------------------------------------------------------------------------------------------
}// end namespace

#endif //STICKYFOOT_LCM_UTILS_HPP
