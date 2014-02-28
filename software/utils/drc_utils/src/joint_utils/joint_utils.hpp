#ifndef __JOINT_UTILS_HPP__
#define __JOINT_UTILS_HPP__

class JointUtils
{
public:
  JointUtils (){

    // Atlas:
    // Note: ordering here MUST match that in AtlasControlTypes.h ***********************
    atlas_joint_names = {"back_bkz", "back_bky", "back_bkx", 
        "neck_ay", "l_leg_hpz", "l_leg_hpx", "l_leg_hpy", 
        "l_leg_kny", "l_leg_aky", "l_leg_akx", "r_leg_hpz", 
        "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", 
        "r_leg_akx", "l_arm_usy", "l_arm_shx", "l_arm_ely", 
        "l_arm_elx", "l_arm_uwy", "l_arm_mwx", "r_arm_usy", 
        "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"};  

    head_joint_names = {"hokuyo_joint","pre_spindle_cal_x_joint", "pre_spindle_cal_y_joint", 
        "pre_spindle_cal_z_joint", "pre_spindle_cal_roll_joint", "pre_spindle_cal_pitch_joint", 
        "pre_spindle_cal_yaw_joint", "post_spindle_cal_x_joint", "post_spindle_cal_y_joint", 
        "post_spindle_cal_z_joint", "post_spindle_cal_roll_joint", "post_spindle_cal_pitch_joint", "post_spindle_cal_yaw_joint" };
    simple_head_joint_names =  {"hokuyo_joint"}; // used in simulation
        
    
    sandia_l_joint_names = {"left_f0_j0","left_f0_j1","left_f0_j2",   "left_f1_j0","left_f1_j1","left_f1_j2",
      "left_f2_j0","left_f2_j1","left_f2_j2",   "left_f3_j0","left_f3_j1","left_f3_j2" };
    sandia_r_joint_names = {"right_f0_j0","right_f0_j1","right_f0_j2",  "right_f1_j0","right_f1_j1","right_f1_j2",
      "right_f2_j0","right_f2_j1","right_f2_j2",  "right_f3_j0","right_f3_j1","right_f3_j2" };
      
    /// iRobot:
    irobot_l_joint_names = {"left_finger[0]/joint_base_rotation", "left_finger[0]/joint_base",
        "left_finger[0]/joint_flex", "left_finger[1]/joint_base_rotation", 
        "left_finger[1]/joint_base", "left_finger[1]/joint_flex",
        "left_finger[2]/joint_base", "left_finger[2]/joint_flex" };
    irobot_r_joint_names = {"right_finger[0]/joint_base_rotation", "right_finger[0]/joint_base",
        "right_finger[0]/joint_flex", "right_finger[1]/joint_base_rotation", 
        "right_finger[1]/joint_base", "right_finger[1]/joint_flex",
        "right_finger[2]/joint_base", "right_finger[2]/joint_flex" };
        
        
    /// Robotiq:
    robotiq_l_joint_names = { "left_finger_1_joint_1", "left_finger_1_joint_2", "left_finger_1_joint_3",
        "left_finger_2_joint_1", "left_finger_2_joint_2", "left_finger_2_joint_3",
        "left_finger_middle_joint_1", "left_finger_middle_joint_2", "left_finger_middle_joint_3",
        "left_palm_finger_1_joint", "left_palm_finger_2_joint"};
    robotiq_r_joint_names = { "right_finger_1_joint_1", "right_finger_1_joint_2", "right_finger_1_joint_3",
        "right_finger_2_joint_1", "right_finger_2_joint_2", "right_finger_2_joint_3",
        "right_finger_middle_joint_1", "right_finger_middle_joint_2", "right_finger_middle_joint_3",
        "right_palm_finger_1_joint", "right_palm_finger_2_joint"};
        

    // Combine all joint names into one variable. Used by Toby in the shapers
    all_joint_names.insert(all_joint_names.end(), atlas_joint_names.begin(), atlas_joint_names.end());
    
    all_joint_names.insert(all_joint_names.end(), head_joint_names.begin(), head_joint_names.end());    
    // all_joint_names.insert(all_joint_names.end(), simple_head_joint_names.begin(), simple_head_joint_names.end()); // skipped as its in the above
    
    all_joint_names.insert(all_joint_names.end(), sandia_l_joint_names.begin(), sandia_l_joint_names.end());
    all_joint_names.insert(all_joint_names.end(), sandia_r_joint_names.begin(), sandia_r_joint_names.end());
    
    all_joint_names.insert(all_joint_names.end(), irobot_l_joint_names.begin(), irobot_l_joint_names.end());
    all_joint_names.insert(all_joint_names.end(), irobot_r_joint_names.begin(), irobot_r_joint_names.end());
    
    all_joint_names.insert(all_joint_names.end(), robotiq_l_joint_names.begin(), robotiq_l_joint_names.end());
    all_joint_names.insert(all_joint_names.end(), robotiq_r_joint_names.begin(), robotiq_r_joint_names.end());

  };
  ~JointUtils() {}
  
  std::vector<std::string> atlas_joint_names, head_joint_names, simple_head_joint_names;    
  std::vector<std::string> irobot_l_joint_names, irobot_r_joint_names;
  std::vector<std::string> sandia_l_joint_names, sandia_r_joint_names;    
  std::vector<std::string> robotiq_l_joint_names, robotiq_r_joint_names; 

  std::vector<std::string> all_joint_names;
private:
};


#endif
