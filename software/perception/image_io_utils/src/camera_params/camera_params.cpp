#include "camera_params.hpp"

CameraParams::CameraParams () {
  fx = 0.0; // probably should set all params to Nan for safety
}

void CameraParams::setParams(BotParam *botparam_, std::string key_prefix ){
  
    width = bot_param_get_int_or_fail(botparam_, (key_prefix+".width").c_str());
    height = bot_param_get_int_or_fail(botparam_,(key_prefix+".height").c_str());
    fx = bot_param_get_double_or_fail(botparam_, (key_prefix+".fx").c_str());
    fy = bot_param_get_double_or_fail(botparam_, (key_prefix+".fy").c_str());
    cx = bot_param_get_double_or_fail(botparam_, (key_prefix+".cx").c_str());
    cy = bot_param_get_double_or_fail(botparam_, (key_prefix+".cy").c_str());
    k1 = bot_param_get_double_or_fail(botparam_, (key_prefix+".k1").c_str());
    k2 = bot_param_get_double_or_fail(botparam_, (key_prefix+".k2").c_str());
    k3 = bot_param_get_double_or_fail(botparam_, (key_prefix+".k3").c_str());
    p1 = bot_param_get_double_or_fail(botparam_, (key_prefix+".p1").c_str());
    p2 = bot_param_get_double_or_fail(botparam_, (key_prefix+".p2").c_str());
}



StereoParams::StereoParams () {
}

void StereoParams::setParams(BotParam *botparam_, std::string key_prefix ){
  std::string left_str = std::string("cameras." + key_prefix + ".left"); 
  left = CameraParams();
  left.setParams(botparam_, left_str  );  

  std::string right_str = std::string("cameras." + key_prefix + ".right"); 
  right = CameraParams();
  right.setParams(botparam_, right_str  );  
  
  
  std::string trans_str = std::string("cameras." + key_prefix + ".translation"); 
  bot_param_get_double_array_or_fail (botparam_, trans_str.c_str(), translation, 3);
  
}
