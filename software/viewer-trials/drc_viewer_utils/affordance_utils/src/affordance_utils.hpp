#ifndef AFFORDANCE_UTILS_HPP
#define AFFORDANCE_UTILS_HPP

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

#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <visualization_utils/angles.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/file_access_utils.hpp>
#include <visualization_utils/gl_draw_utils.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/affordance_t.hpp"
#include "lcmtypes/drc/affordance_plus_t.hpp"
#include <lcmtypes/bot_core.h>

using namespace std;

namespace visualization_utils
{

   struct OtdfInstanceStruc {

      OtdfInstanceStruc()
      {
       otdf_instance_viz_object_sync = true; 
        is_melded= false;
      };
      
       ~OtdfInstanceStruc()
      {

      };

      std::string otdf_type;
      int uid;
      boost::shared_ptr<otdf::ModelInterface> _otdf_instance;
      boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_object;
      // Each object has its own collision detector. Makes it easy to handle inclusion
      // and deletion of objects
      boost::shared_ptr<collision::Collision_Detector> _collision_detector;  
     
      // filename of model used to generate points or triangles (alternative to points/triangles in aff message)
      std::string modelfile;

      // bounding box info relative to object
      Eigen::Vector3f boundingBoxXYZ;
      Eigen::Vector3f boundingBoxRPY;
      Eigen::Vector3f boundingBoxLWH;
      
      // otdf instance is in sync with aff server. 
      // Turn off if one needs to visualize state changes before committing to aff server.    
      bool otdf_instance_viz_object_sync;
      bool is_melded; // if melded via a sticky hand, the affordance is tracked via EST_ROBOT_STATE

  };   
 
  
}//end_namespace


#endif //AFFORDANCE_UTILS_HPP

