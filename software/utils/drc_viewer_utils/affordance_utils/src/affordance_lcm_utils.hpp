#ifndef AFFORDANCE_LCM_UTILS_HPP
#define AFFORDANCE_LCM_UTILS_HPP
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace std;
using namespace boost;
//using namespace visualization_utils;

namespace visualization_utils
{
 //----------------------------------------------------------------------------------------------------------   
  inline static bool get_aff_indexed_traj_opt_constraint(int64_t last_state_msg_timestamp, string robot_name,
                                                          map<string, vector<KDL::Frame> > &ee_frames_map,
                                                          map<string, vector<drc::affordance_index_t> > &ee_frame_affindices_map,
                                                          drc::aff_indexed_traj_opt_constraint_t &trajmsg)
  {
    trajmsg.utime = last_state_msg_timestamp;
    trajmsg.robot_name = robot_name;

    int num_links = 0;

    // N ee's and K keyframes
    for(map<string,vector<KDL::Frame> >::iterator it = ee_frames_map.begin(); it!=ee_frames_map.end(); it++) { 
        string ee_name = it->first;
        vector<KDL::Frame> ee_frames  = it->second;
        map<string, vector<drc::affordance_index_t> >::iterator ts_it = ee_frame_affindices_map.find(it->first);
        if(ts_it == ee_frame_affindices_map.end()){
            cerr << "ERROR: No Aff index found for ee " << it->first << endl;      
            return false;
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
    return true;         
  }

  //----------------------------------------------------------------------------------------------------------        
  inline static bool get_traj_opt_constraint(int64_t last_state_msg_timestamp, string robot_name,
                                                                    map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                                    map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                                    map<string, vector<double> > &joint_pos_map,
                                                                    map<string, vector<int64_t> > &joint_pos_timestamps_map,
                                                                    drc::traj_opt_constraint_t &trajmsg)
  {
                                                              
    trajmsg.utime = last_state_msg_timestamp;
    trajmsg.robot_name = robot_name;

    int num_links = 0;
    // N ee's and K keyframes
    for(map<string,vector<KDL::Frame> >::iterator it = ee_frames_map.begin(); it!=ee_frames_map.end(); it++)
    { 
        string ee_name = it->first;
        vector<KDL::Frame> ee_frames  = it->second;
        map<string, vector<int64_t> >::iterator ts_it = ee_frame_timestamps_map.find(it->first);
        if(ts_it == ee_frame_timestamps_map.end()){
            cerr << "ERROR: No Timestamp found for ee " << it->first << endl;      
            return false;
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
            return false;
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
    return true;
  }
 //----------------------------------------------------------------------------------------------------------         
      

}// end namespace

#endif //AFFORDANCE_LCM_UTILS_HPP
