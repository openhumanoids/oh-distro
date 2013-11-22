#ifndef STICKY_HAND_UTILS_HPP
#define STICKY_HAND_UTILS_HPP

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string> 
#include <vector>
#include <math.h>
#include <fstream>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <errno.h>
#include <dirent.h>
#include <Eigen/Dense>
#include <path_util/path_util.h>
#include <visualization_utils/affordance_utils/affordance_utils.hpp>
#include <visualization_utils/affordance_utils/AffordanceCollectionManager.hpp>

using namespace std;

namespace visualization_utils
{

  struct StickyHandStruc {

      StickyHandStruc()
      {
       grasp_status = 0;
       motion_trail_log_enabled = true;
       is_melded= false;
       squeeze_factor = 1.0;
       is_conditional=false;
       cond_type = 0;
      };
      
       ~StickyHandStruc()
      {

      };
      
      boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_hand;
      boost::shared_ptr<collision::Collision_Detector> _collision_detector;
      string object_name;
      string geometry_name;
      
      // reflects the enum in lcmtypes desired_grasp_state_t and grasp_opt_control_t 
      enum {
       SANDIA_LEFT=0, SANDIA_RIGHT, SANDIA_BOTH, IROBOT_LEFT, IROBOT_RIGHT, IROBOT_BOTH,ROBOTIQ_LEFT, ROBOTIQ_RIGHT, ROBOTIQ_BOTH, INERT_LEFT, INERT_RIGHT, INERT_BOTH
      };
      
      int hand_type; //const int16_t SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5,ROBOTIQ_LEFT=6, ROBOTIQ_RIGHT=7, ROBOTIQ_BOTH=8,INERT=9; 
      KDL::Frame T_geometry_hand; // this is stored in obj frame
      std::vector<std::string> joint_name;
      std::vector<double> joint_position;
      double squeeze_factor;
      bool is_melded;
      KDL::Frame optimized_T_geometry_hand; // store as backup when melded. Retains the ability to unmeld.
      std::vector<double> optimized_joint_position; // store as backup when melded. Retains the ability to unmeld.
      int uid;
      int opt_status;//RUNNING=0, SUCCESS=1, FAILURE=2;
      int grasp_status;//CANDIDATE=0,COMMITTED=1;
      int partial_grasp_status; 
      bool motion_trail_log_enabled;
      bool is_conditional;
      enum {
       GT=0, LT
      };
      int cond_type; // GT,LT TODO: Support for ranges? A list of conditions?
      std::string conditioned_parent_joint_name;
      double conditioned_parent_joint_val;
  };   

   
  //===============================================================================
  //  UTILS  for StickyHandStruc

  bool is_sticky_hand_condition_active(const StickyHandStruc &hand_struc,boost::shared_ptr<visualization_utils::AffordanceCollectionManager>  &affCollectionManager);
  void get_min_dimension(boost::shared_ptr<otdf::Geometry> &link_geom, std::string &min_dimension_tag);
  
  // Utils for seeding sticky hands in the viewer
  void get_user_specified_hand_approach(Eigen::Vector3d objectframe_finger_dir, Eigen::Vector3d from, Eigen::Vector3d to, boost::shared_ptr<otdf::Geometry> &link_geom, KDL::Frame &T_objectgeometry_lhand, KDL::Frame &T_objectgeometry_rhand,bool is_sandia);
  void get_hand_approach(Eigen::Vector3d from, Eigen::Vector3d to, boost::shared_ptr<otdf::Geometry> &link_geom, KDL::Frame &T_objectgeometry_lhand,KDL::Frame &T_objectgeometry_rhand,bool is_sandia);
  bool get_stickyhand_init_positions(string& object_name, string& geometry_name,OtdfInstanceStruc &obj,
                         Eigen::Vector3f &ray_start,Eigen::Vector3f &ray_hit,Eigen::Vector3f &ray_hit_drag, bool dragging,
                         KDL::Frame &T_graspgeometry_lhandinitpos, KDL::Frame &T_graspgeometry_rhandinitpos, bool is_sandia);
  
  // sticky hand presets interms of eigen grasps                       
  void set_joint_position(StickyHandStruc &hand_struc,std::string &name, double val);
  void get_eigen_grasp_types(std::vector<std::string> &_names);
  void set_eigen_grasp(StickyHandStruc &hand_struc,int eig_grasp_type);
  
  bool get_hand_palm_link_name(int hand_type,std::string &ee_name);
  bool get_hand_name(int hand_type,std::string &hand_name);
  bool get_hand_urdf_file_name_and_ext(int hand_type,std::string &file_name,std::string &ext);

}//end_namespace


#endif //STICKY_HAND_UTILS_HPP

