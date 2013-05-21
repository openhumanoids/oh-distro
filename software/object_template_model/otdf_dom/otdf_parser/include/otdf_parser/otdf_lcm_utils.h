
#ifndef OTDF_LCM_UTILS_H
#define OTDF_LCM_UTILS_H

#include <iostream>
#include <fstream>

#include <errno.h>
#include <dirent.h>

#include <tinyxml.h>

#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include <otdf_interface/model.h>
#include <path_util/path_util.h>
#include "otdf_parser/otdf_parser.h"
#include "otdf_parser/otdf_urdf_converter.h"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

namespace otdf{

  //------------------
  // Affordance Msg to msg. Where should this message be?
   static bool AffordanceLcmMsgToUrdfString(const drc::affordance_t &msg, std::string &urdf_xml_string)
   {
    std::string filename = msg.otdf_type;
    std::string otdf_xml_string;
   
    if(!get_xml_string_from_file(filename, otdf_xml_string)){
      std::cerr << "ERROR: file extraction failed" << std::endl; 
      return false; // file extraction failed
    }
        
 
    boost::shared_ptr<ModelInterface> otdf_instance = parseOTDF(otdf_xml_string); 
    otdf_instance->setParam("x",msg.origin_xyz[0]);   
    otdf_instance->setParam("y",msg.origin_xyz[1]);  
    otdf_instance->setParam("z",msg.origin_xyz[2]); 
    otdf_instance->setParam("roll",msg.origin_rpy[0]);   
    otdf_instance->setParam("pitch",msg.origin_rpy[1]);  
    otdf_instance->setParam("yaw",msg.origin_rpy[2]); 
    for(int i=0;i<msg.nparams;i++){
      otdf_instance->setParam(msg.param_names[i],msg.params[i]); 
    }
    for(int i=0;i<msg.nstates;i++){
      double pos, vel;
      pos = msg.states[i];
      vel =0;
      otdf_instance->setJointState(msg.state_names[i],pos,vel); 
    }

    otdf_instance->update();
    urdf_xml_string=convertObjectInstanceToURDFstring(otdf_instance);
    return true;
   };
   
   
  // this function should go into otdf_utils library
  // usage 
  static int get_filenames_from_otdf_models_dir (std::vector<std::string> &files)
  {
      std::string dir =  std::string(getModelsPath()) + "/otdf/"; 
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          std::cout << "Error(" << errno << ") opening " << dir << std::endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =std::string(dirp->d_name);
        if(fn.substr(fn.find_last_of(".") + 1) == "otdf") 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      closedir(dp);
      return 0;
  };


} // end namespace

#endif

