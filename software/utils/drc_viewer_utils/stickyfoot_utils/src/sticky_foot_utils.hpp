#ifndef STICKY_FOOT_UTILS_HPP
#define STICKY_FOOT_UTILS_HPP

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

using namespace std;

namespace visualization_utils
{

  struct StickyFootStruc {

      StickyFootStruc()
      {
       motion_trail_log_enabled = true;
       is_melded= false;
      };
      
       ~StickyFootStruc()
      {

      };
      boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_foot;
      boost::shared_ptr<collision::Collision_Detector> _collision_detector;
      string object_name;
      string geometry_name; 
      int foot_type; //LEFT=0, RIGHT=1;
      KDL::Frame T_geometry_foot; // this is stored in obj frame
      std::vector<std::string> joint_name;
      std::vector<double> joint_position;
      bool is_melded;
      KDL::Frame optimized_T_geometry_foot; // store as backup when melded. Retains the ability to unmeld.
      std::vector<double> optimized_joint_position; // store as backup when melded. Retains the ability to unmeld.
      int uid;
      bool motion_trail_log_enabled;
     // int opt_status;
  }; 

}//end_namespace


#endif //STICKY_FOOT_UTILS_HPP

