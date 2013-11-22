#ifndef STICKYFOOT_LCM_UTILS_HPP
#define STICKYFOOT_LCM_UTILS_HPP
#include "sticky_foot_utils.hpp"
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace std;
using namespace boost;
//using namespace visualization_utils;


namespace visualization_utils
{
  // lcm util functions that parses constraints stored in sticky_foot_struc into the required form for reachoing/manip/endpose planners 

    drc::ee_goal_t get_eegoal_to_stickyfoot(StickyFootStruc &sticky_foot_struc,
                                           string robot_name,
                                           string root_name, 
                                           string ee_name,
                                           KDL::Frame &T_world_geometry,
                                           bool reach_flag);
    
    //----------------------------------------------------------------------------------------------------------    
   drc::traj_opt_constraint_t get_desired_foot_motion_traj_constraint(int64_t last_state_msg_timestamp,
                                                                      string robot_name,
                                                                      KDL::Frame &T_world_object,
                                                                      StickyFootStruc &sticky_foot_struc, 
                                                                      string ee_name);
    
    //----------------------------------------------------------------------------------------------------------
        
    void get_endpose_search_constraints_from_sticky_foot(StickyFootStruc &footstruc,OtdfInstanceStruc& obj, 
                                                        KDL::Frame& T_world_body_desired, 
                                                        bool to_future_state,
                                                        bool end_state_only,
                                                        map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                        map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                        map<string, vector<double> > &joint_pos_map,
                                                        map<string, vector<int64_t> > &joint_pos_timestamps_map);
//----------------------------------------------------------------------------------------------------------  
}// end namespace

#endif //STICKYFOOT_LCM_UTILS_HPP
