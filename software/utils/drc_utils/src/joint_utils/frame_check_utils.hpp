#ifndef __FRAME_CHECK_UTILS_HPP__
#define __FRAME_CHECK_UTILS_HPP__

/*
SCAN has parent: POST_SPINDLE <fixed>
POST_SPINDLE has parent: PRE_SPINDLE <from multisense as PRE_SPINDLE_TO_POST_SPINDLE>
PRE_SPINDLE has parent: CAMERA_LEFT <fixed>
CAMERA_LEFT has parent: head <fixed>
head has parent: body  <from FK in joint2frames as BODY_TO_HEAD>
body has parent: local <by the estimator as POSE_BODY>
*/

class FrameCheckUtils
{
public:
  FrameCheckUtils (){
    local_to_head_frame_valid_ = false;

  };
  ~FrameCheckUtils() {}
  
  
  // Check if the SCAN frames have been updated at all.
  bool isLocalToScanValid(BotFrames* botframes){

    if (local_to_head_frame_valid_){
      return true; 
    }else{ // check each updating frame has been updated already.      
      int64_t last_update_utime_1, last_update_utime_2, last_update_utime_3;

      if (!bot_frames_get_trans_latest_timestamp(botframes, "PRE_SPINDLE", "POST_SPINDLE", &last_update_utime_1)){
        return false; 
      }
      if (!bot_frames_get_trans_latest_timestamp(botframes, "head", "body", &last_update_utime_2)){
        return false; 
      }
      if (!bot_frames_get_trans_latest_timestamp(botframes, "body", "local", &last_update_utime_3)){
        return false; 
      }
      
      std::cout << "PRE_SPINDLE" << " to " << "POST_SPINDLE" << " last updated " << last_update_utime_1 << "\n";
      std::cout << "head" << " to " << "body" << " last updated " << last_update_utime_2 << "\n";
      std::cout << "body" << " to " << "local" << " last updated " << last_update_utime_3 << "\n";
      
      if (( (last_update_utime_1 > 0) && (last_update_utime_2 > 0) )  && (last_update_utime_3 > 0) ){
        local_to_head_frame_valid_ = true;
        return true;
      }else{
        return false;
      }
    }
  }
  
  
  // Check if the SCAN frames are more recent than the scan utime (to avoid extrapolation):
  bool isLocalToScanUpdated(BotFrames* botframes, int64_t scan_utime){
    int64_t last_update_utime_1, last_update_utime_2, last_update_utime_3;
    if (!bot_frames_get_trans_latest_timestamp(botframes, "PRE_SPINDLE", "POST_SPINDLE", &last_update_utime_1)){
      return false; 
    }
    if (!bot_frames_get_trans_latest_timestamp(botframes, "head", "body", &last_update_utime_2)){
      return false; 
    }
    if (!bot_frames_get_trans_latest_timestamp(botframes, "body", "local", &last_update_utime_3)){
      return false; 
    }
    
    std::cout << "PRE_SPINDLE" << " to " << "POST_SPINDLE" << " last updated " << last_update_utime_1 << "\n";
    std::cout << "head" << " to " << "body" << " last updated " << last_update_utime_2 << "\n";
    std::cout << "body" << " to " << "local" << " last updated " << last_update_utime_3 << "\n";
    
    if (( (last_update_utime_1 > scan_utime) && (last_update_utime_2 > scan_utime) )  && (last_update_utime_3 > scan_utime) ){
      local_to_head_frame_valid_ = true;
      return true;
    }else{
      return false;
    }
  }  
  
  
private:
  
  bool local_to_head_frame_valid_;
};


#endif
