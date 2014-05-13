#include "camera_params.hpp"

CameraParams::CameraParams () {
  fx = 0.0; // probably should set all params to Nan for safety
}

void CameraParams::setParams(BotParam *botparam_, std::string key_prefix ){
  width = bot_param_get_int_or_fail(botparam_, (key_prefix+".intrinsic_cal.width").c_str());
  height = bot_param_get_int_or_fail(botparam_,(key_prefix+".intrinsic_cal.height").c_str());
  
  double vals[5];
  bot_param_get_double_array_or_fail(botparam_, (key_prefix+".intrinsic_cal.pinhole").c_str(), vals, 5);
  fx = vals[0];
  fy = vals[1];
  cx = vals[3];
  cy = vals[4];
  if (3 == bot_param_get_double_array(botparam_, (key_prefix+".intrinsic_cal.distortion_k").c_str(), vals, 3)) {
    k1 = vals[0];
    k2 = vals[1];
    k3 = vals[2];
  }
  if (2 == bot_param_get_double_array(botparam_, (key_prefix+".intrinsic_cal.distortion_p").c_str(), vals, 2)) {
    p1 = vals[0];
    p2 = vals[1];
  }
}

StereoParams::StereoParams () {
}

void StereoParams::setParams(BotParam *botparam_, std::string key_prefix ){
  std::string left_str = std::string("cameras." + key_prefix + "_LEFT"); 
  left = CameraParams();
  left.setParams(botparam_, left_str  );  

  std::string right_str = std::string("cameras." + key_prefix + "_RIGHT"); 
  right = CameraParams();
  right.setParams(botparam_, right_str  );  
  
  std::string trans_str = std::string("cameras." + key_prefix + ".translation"); 
  bot_param_get_double_array_or_fail (botparam_, trans_str.c_str(), translation, 3);
  std::string rot_str = std::string("cameras." + key_prefix + ".rotation"); 
  bot_param_get_double_array_or_fail (botparam_, rot_str.c_str(), rotation, 9); 
}