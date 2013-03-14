// mfallon sept 2012

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc.hpp"

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/bot_param_update_t.h>
#include <lcmtypes/drc_lcmtypes.h>

//#include <lcmtypes/perception_pointing_vector_t.h>
#include "tld_lcmgl.hpp"

#include <ConciseArgs>
#include <drc_utils/Clock.hpp>

using namespace std;
//using namespace cv;

tld_lcmgl::tld_lcmgl(boost::shared_ptr<lcm::LCM> &lcm_):
          lcm_(lcm_){
            
  param = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  if (param == NULL) {
    fprintf(stderr, "Couldn't get bot param from server.\n");
    exit(-1);
  }
  lcmgl_ = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "tld_lcmgl");
  frames = bot_frames_get_global (lcm_->getUnderlyingLCM(), param);
  drc::Clock::instance()->setLcm(lcm_);

  lcm_->subscribe("SEEK_GOAL",&tld_lcmgl::on_seek_goal,this);  
  lcm_->subscribe("OBJECT_BEARING",&tld_lcmgl::on_pointing_vector,this);  

  seek_timeout_ = 0;
  seek_utime_ = 0;
  seek_type_ = 0;
}


void tld_lcmgl::on_seek_goal(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& channel,
                                   const  drc::seek_goal_timed_t* msg){
  cout << "got seek_goal @ " << msg->utime << "\n";
  seek_timeout_ = msg->timeout;
  seek_utime_ = msg->utime;
  seek_type_ = msg->type;
}



void tld_lcmgl::on_pointing_vector(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& channel,
                                   const  perception::pointing_vector_t* msg){
  int64_t curr_clock = drc::Clock::instance()->getCurrentTime();
  if (curr_clock < (seek_utime_ + seek_timeout_)){
    //cout << "follow vector\n";
      
    // Convert vector into point very far away
    double scale = 100.0;
    double goal_pos[]={scale*msg->vec[0], scale*msg->vec[1],scale* msg->vec[2]};
    
    int status;
    Eigen::Isometry3d cam_to_frame;

    if (seek_type_ == DRC_SEEK_GOAL_TIMED_T_VISUAL_GLOBAL){ // Send a Global goal:
      status = frames_cpp->get_trans_with_utime( frames , "CAMERA",  "local", msg->utime, cam_to_frame);
    }else if (seek_type_ == DRC_SEEK_GOAL_TIMED_T_VISUAL_RELATIVE){ // Send a relative goal:
      status = frames_cpp->get_trans_with_utime( frames , "CAMERA",  "body", msg->utime, cam_to_frame);
    }else{
      std::cerr << "Ignoring this pointing_vector - don't understand type\n";
      return; 
    }
    if (0 == status) {
      std::cerr << "SensorDataReceiver: cannot get transform from CAMERA to body" << std::endl;
      return;
    }
      
    Eigen::Vector4d pos_vec= Eigen::Vector4d(goal_pos[0], goal_pos[1], goal_pos[2], 1);
    Eigen::Vector4d pos_vec_out =  cam_to_frame*pos_vec;
    Eigen::Vector4d null_vec= Eigen::Vector4d(0,0,0, 1);
    Eigen::Vector4d null_vec_out =  cam_to_frame*null_vec;

    
    // Set goal:
    drc::nav_goal_timed_t msgout;
    msgout.utime = msg->utime; //bot_timestamp_now();
    msgout.timeout = (int64_t) 1E6*1; //self->goal_timeout;
    msgout.robot_name = "wheeled_atlas"; // this should be set from robot state message
    msgout.goal_pos.translation.x = pos_vec_out[0];
    msgout.goal_pos.translation.y = pos_vec_out[1];
    msgout.goal_pos.translation.z = 0;
    double rpy[] = {0,0,0}; //self->theta
    double quat_out[4];
    bot_roll_pitch_yaw_to_quat(rpy, quat_out); // its in w,x,y,z format
    msgout.goal_pos.rotation.w = 1;
    msgout.goal_pos.rotation.x = 0;
    msgout.goal_pos.rotation.y = 0;
    msgout.goal_pos.rotation.z = 0;
    
    if (seek_type_==DRC_SEEK_GOAL_TIMED_T_VISUAL_GLOBAL){ // Send a Global goal:
      fprintf(stderr, "Following Vector | Sending NAV_GOAL_TIMED\n");
      lcm_->publish("NAV_GOAL_TIMED", &msgout); 
    }else if (seek_type_ == DRC_SEEK_GOAL_TIMED_T_VISUAL_RELATIVE){ // Send a relative goal:
      fprintf(stderr, "Following Vector | Sending RELATIVE_NAV_GOAL_TIMED\n");
      lcm_->publish("RELATIVE_NAV_GOAL_TIMED", &msgout);       
    }

    bot_lcmgl_push_matrix(lcmgl_);
    bot_lcmgl_line_width(lcmgl_,3.0);  
    bot_lcmgl_color3f(lcmgl_, 1, 0, 0); // cyan
    bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);  
    bot_lcmgl_vertex3f(lcmgl_, null_vec_out[0],  null_vec_out[1], null_vec_out[2]);
    bot_lcmgl_vertex3f(lcmgl_, pos_vec_out[0],  pos_vec_out[1], pos_vec_out[2]);
    bot_lcmgl_end(lcmgl_);
    bot_lcmgl_pop_matrix(lcmgl_);     
    bot_lcmgl_switch_buffer(lcmgl_);  
  }else{
      cout << "Ignore Pointing Vector | Goal has expired\n";
  }
}

int main(int argc, char ** argv) {
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;  
  
  tld_lcmgl app(lcm);
  
  while(0 == lcm->handle()); 
  
  return 0;
}
