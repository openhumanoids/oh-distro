#ifndef STICKYHAND_LCM_UTILS_HPP
#define STICKYHAND_LCM_UTILS_HPP

#include "sticky_hand_utils.hpp"
#include "lcmtypes/drc/desired_grasp_state_t.hpp"
#include "lcmtypes/drc/ee_goal_t.hpp"
#include "lcmtypes/drc/traj_opt_constraint_t.hpp"

using namespace std;
using namespace boost;
//using namespace visualization_utils;

namespace visualization_utils
{
  // lcm util functions that 
  // 1) parses grasp state stored in sticky_hand_struc into the required lcm msgs for grasp controllers
  // 2) parses constraints stored in sticky_hand_struc into the required form for reachoing/manip/endpose planners 

  //----------------------------------------------------------------------------------------------------------          
  drc::desired_grasp_state_t get_grasp_state(int64_t last_state_msg_timestamp, string robot_name,
                                              StickyHandStruc &sticky_hand_struc,string ee_name,
                                              KDL::Frame &T_world_geometry,  bool grasp_flag,
                                              bool power_flag,bool squeeze_flag, double squeeze_amount);
  //----------------------------------------------------------------------------------------------------------   
  drc::desired_grasp_state_t  get_partial_grasp_state(int64_t last_state_msg_timestamp, string robot_name,
                                                              StickyHandStruc &sticky_hand_struc,string ee_name, 
                                                              KDL::Frame &T_world_geometry,int grasp_state);
  //----------------------------------------------------------------------------------------------------------       
  drc::ee_goal_t get_eegoal_to_sticky_hand(StickyHandStruc &sticky_hand_struc,
                                           string robot_name,
                                           string root_name,
                                           string ee_name,
                                           KDL::Frame &T_world_geometry,
                                           bool reach_flag);
  //----------------------------------------------------------------------------------------------------------  
  drc::traj_opt_constraint_t get_desired_hand_motion_traj_constraint(int64_t last_state_msg_timestamp, 
                                                                    string robot_name,
                                                                    KDL::Frame &T_world_object,
                                                                    StickyHandStruc &sticky_hand_struc,
                                                                    string ee_name);
  //-------------------------------------------------------------------------------------------
  void get_endpose_search_constraints_from_sticky_hand(StickyHandStruc &handstruc,
                                                      OtdfInstanceStruc& obj,
                                                      KDL::Frame& T_world_body_desired,
                                                      bool to_future_state,
                                                      bool end_state_only,
                                                      map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                      map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                      map<string, vector<double> > &joint_pos_map,
                                                      map<string, vector<int64_t> > &joint_pos_timestamps_map);
  //---------------------------------------------------------------------------------------------------------- 
  void get_motion_constraints_from_sticky_hand(StickyHandStruc &handstruc,OtdfInstanceStruc& obj, 
                                                  bool is_retracting, bool is_cyclic,
                                                  map<string, vector<KDL::Frame> > &ee_frames_map, 
                                                  map<string, vector<int64_t> > &ee_frame_timestamps_map,
                                                  map<string, vector<double> > &joint_pos_map,
                                                  map<string, vector<int64_t> > &joint_pos_timestamps_map,
                                                  double retracting_offset);

  //---------------------------------------------------------------------------------------------------------- 
           
}// end namespace

#endif //STICKYHAND_LCM_UTILS_HPP
