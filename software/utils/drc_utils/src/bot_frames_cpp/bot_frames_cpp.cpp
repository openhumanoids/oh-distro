#include "bot_frames_cpp.hpp"

#include <iostream>

namespace bot {

frames::frames(){
  // unsafe initialisation
  // included for backward compatiabilty
  // TODO: deprecate usage else where
};


frames::frames(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
};

frames::frames(boost::shared_ptr<lcm::LCM> &lcm_, BotParam *botparam_ ): lcm_(lcm_){
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
};

frames::frames(BotParam *botparam_ ){
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
};

frames::frames(BotFrames *frames ){
  botframes_= frames;
};

int frames::get_trans_with_utime(std::string from_frame, std::string to_frame, 
                                 int64_t utime, Eigen::Isometry3d & mat){
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( botframes_, from_frame.c_str(),  to_frame.c_str(), utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }  

  return status;
}



/// Older, Depreciated:
int frames::get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d & mat){
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( bot_frames, from_frame,  to_frame, utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }  

  return status;
}

Eigen::Isometry3d frames::get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime){
  Eigen::Isometry3d mat;
  get_trans_with_utime(bot_frames, from_frame, to_frame, utime, mat);
  return mat;
}




}
