#include "sticky_foot_lcm_utils.hpp"

using namespace std;
using namespace boost;

namespace visualization_utils
{

    drc::ee_goal_t get_eegoal_to_stickyfoot(StickyFootStruc &sticky_foot_struc, string robot_name,string root_name, string ee_name, KDL::Frame &T_world_geometry,bool reach_flag)
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
   drc::traj_opt_constraint_t get_desired_foot_motion_traj_constraint(int64_t last_state_msg_timestamp, string robot_name,
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
        
    void get_endpose_search_constraints_from_sticky_foot(StickyFootStruc &footstruc,OtdfInstanceStruc& obj, KDL::Frame& T_world_body_desired, bool to_future_state,
                                                      bool end_state_only,
                                                      map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                      map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                      map<string, vector<double> > &joint_pos_map,
                                                      map<string, vector<int64_t> > &joint_pos_timestamps_map)
    {
          string ee_name;
          if(footstruc.foot_type==0)
              ee_name ="l_foot";    
          else if(footstruc.foot_type==1)   
              ee_name ="r_foot";            
          else
              cout << "unknown foot_type in get_endpose_search_constraints_from_sticky_foot\n";   

          // if ee_name already exists in ee_frames_map, redundant ee_frames
          // e.g two right sticky hands on the same object.
          map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
          if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
              return;   
          }

          //======================     
          KDL::Frame  T_geometry_foot = footstruc.T_geometry_foot;
        
          KDL::Frame T_world_object = KDL::Frame::Identity();
          KDL::Frame T_world_geometry = KDL::Frame::Identity();
          if(!to_future_state){
            T_world_object = obj._gl_object->_T_world_body;
            // the object might have moved.
            if(!obj._gl_object->get_link_geometry_frame(string(footstruc.geometry_name),T_world_geometry))
                cerr << " failed to retrieve " << footstruc.geometry_name<<" in object " << footstruc.object_name <<endl;
            }
            else {
              T_world_object = obj._gl_object->_T_world_body_future;
              // the object might have moved.
              if(!obj._gl_object->get_link_geometry_future_frame(string(footstruc.geometry_name),T_world_geometry))
                cerr << " failed to retrieve " << footstruc.geometry_name<<" in object " << footstruc.object_name <<endl;        
          }

          int num_frames = 1;
          if((to_future_state)&&(!end_state_only))
            num_frames = footstruc._gl_foot->_desired_body_motion_history.size();
          
          vector<KDL::Frame> T_world_ee_frames;
          vector<int64_t> frame_timestamps;
          for(uint i = 0; i < (uint) num_frames; i++) {
          
             KDL::Frame T_world_ee;      
              if((to_future_state)&&(!end_state_only)){
                 KDL::Frame T_object_foot = footstruc._gl_foot->_desired_body_motion_history[i];
                //obj._gl_object->_T_world_body is the frame in which desired body is accumulated.
                 KDL::Frame T_world_foot = obj._gl_object->_T_world_body*T_object_foot;
                 T_world_ee = T_world_foot;//T_world_palm ; TODO: Eventually will be in object frame     
              }
              else
                T_world_ee = T_world_geometry*T_geometry_foot;
                  
              KDL::Vector footframe_offset;
              footframe_offset[0]=0.0;footframe_offset[1]=0;footframe_offset[2]=0;
              KDL::Vector worldframe_offset=T_world_ee.M*footframe_offset;
              T_world_ee.p += worldframe_offset;                    
              
              T_world_ee_frames.push_back(T_world_ee);
              int64_t timestamp=(int64_t)i*1000000;
              frame_timestamps.push_back(timestamp);   
          }
          //===================         

          ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
          ee_frame_timestamps_map.insert(make_pair(ee_name, frame_timestamps));   
          // footstruc.joint_name;  
          // footstruc.joint_position; 
          // posture constraints for transitions only
          
            for(uint k = 0; k< (uint) footstruc.joint_name.size(); k++) 
            {
              std::string joint_name = footstruc.joint_name[k];
              vector<double> joint_pos;
              vector<int64_t> joint_pos_timestamps;
              uint i=0;
              joint_pos.push_back(footstruc.joint_position[k]);
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
               
    } // end get_endpose_search_constraints_from_sticky_foot  
//----------------------------------------------------------------------------------------------------------  
}// end namespace

