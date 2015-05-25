// Convert between kintinuous and mit formats
// input: KINECT_DATA
// output: KINECT_FRAME

#include <stdio.h>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/openni.hpp"
#include "lcmtypes/kinect.hpp"
class rgb_tool{
  public:
    rgb_tool(lcm::LCM* &lcm_);
    ~rgb_tool(){}
    
  private:
    lcm::LCM* lcm_;
    
    void kintinuousHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  openni::frame_msg_t* msg);
    void kinectHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  kinect::frame_msg_t* msg);
    
    int decimate_;
};    

rgb_tool::rgb_tool(lcm::LCM* &lcm_): lcm_(lcm_){

  lcm::Subscription* sub1 = lcm_->subscribe( "KINECT_DATA",&rgb_tool::kintinuousHandler,this);
  sub1->setQueueCapacity(1);
  // lcm::Subscription* sub2 =lcm_->subscribe( "KINECT_FRAME",&rgb_tool::kinectHandler,this);
  // sub2->setQueueCapacity(1);
}

void rgb_tool::kintinuousHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  openni::frame_msg_t* msg){
  
  if (1==0){
    bot_core::image_t lcm_img;
    lcm_img.utime =msg->image.timestamp;
    lcm_img.width =msg->image.width;
    lcm_img.height =msg->image.height;
    lcm_img.nmetadata =0;
    int n_colors = 3;
    lcm_img.row_stride=n_colors*msg->image.width;
    if (msg->image.image_data_format == openni::image_msg_t::VIDEO_RGB){
      lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
      lcm_img.size = msg->image.image_data_nbytes;
      lcm_img.data = msg->image.image_data;
    }else if (msg->image.image_data_format == openni::image_msg_t::VIDEO_RGB_JPEG){
      lcm_img.data = msg->image.image_data;
      lcm_img.size = msg->image.image_data_nbytes;
      lcm_img.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    }else{
      std::cout << "Format not recognized: " << msg->image.image_data_format << std::endl;
    }
    lcm_->publish("KINECT_RGB", &lcm_img);
  }


  kinect::image_msg_t img;
  img.timestamp =msg->image.timestamp;
  img.width =msg->image.width;
  img.height =msg->image.height;
  img.image_data_format =msg->image.image_data_format;
  img.image_data_nbytes =msg->image.image_data_nbytes;
  img.image_data =msg->image.image_data;
  //lcm_->publish("KINECT_RGB_ONLY", &img);

  kinect::depth_msg_t depth;
  depth.timestamp =msg->depth.timestamp;
  depth.width =msg->depth.width;
  depth.height =msg->depth.height;
  depth.depth_data_format =kinect::depth_msg_t::DEPTH_MM;
  depth.depth_data_nbytes =msg->depth.depth_data_nbytes;
  depth.depth_data =msg->depth.depth_data;
  depth.compression = msg->depth.compression;
  depth.uncompressed_size = 480*640*2;//msg->depth.uncompressed_size; // the original value was not properly filled in
  //lcm_->publish("KINECT_DEPTH_ONLY", &depth);

  kinect::frame_msg_t out;
  out.timestamp = msg->timestamp;
  out.image = img;
  out.depth = depth;
  lcm_->publish("KINECT_FRAME", &out);

  // missing values from kintinuous:
  // output values: 32616, 0, -111, 0 - not interesting
  //std::cout << msg->depth.pixel_offset[0] << ", " << msg->depth.pixel_offset[1] << ", "
  //          << (int) msg->depth.viewpoint << ", " << msg->depth.focal_length << "\n";
}


void rgb_tool::kinectHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  kinect::frame_msg_t* msg){
  if (1==0){
    bot_core::image_t lcm_img;
    lcm_img.utime =msg->image.timestamp;
    lcm_img.width =msg->image.width;
    lcm_img.height =msg->image.height;
    lcm_img.nmetadata =0;
    int n_colors = 3;
    lcm_img.row_stride=n_colors*msg->image.width;
    if (msg->image.image_data_format == kinect::image_msg_t::VIDEO_RGB){
      lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
      lcm_img.size = msg->image.image_data_nbytes;
      lcm_img.data = msg->image.image_data;
    }else if (msg->image.image_data_format == kinect::image_msg_t::VIDEO_RGB_JPEG){
      lcm_img.data = msg->image.image_data;
      lcm_img.size = msg->image.image_data_nbytes;
      lcm_img.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    }else{
      std::cout << "Format not recognized: " << msg->image.image_data_format << std::endl;
    }
    lcm_->publish("KINECT_RGB", &lcm_img);
  }

  openni::image_msg_t img;
  img.timestamp =msg->image.timestamp;
  img.width =msg->image.width;
  img.height =msg->image.height;
  //img.pixel_offset[0] = 32616;
  //img.pixel_offset[1] = 0;
  //img.viewpoint = -111;

  img.image_data_format =msg->image.image_data_format;
  img.image_data_nbytes =msg->image.image_data_nbytes;
  img.image_data =msg->image.image_data;
  //lcm_->publish("KINECT_RGB_ONLY", &img);

  openni::depth_msg_t depth;
  depth.timestamp =msg->depth.timestamp;
  depth.width =msg->depth.width;
  depth.height =msg->depth.height;
  //depth.depth_data_format =openni::depth_msg_t::DEPTH_MM;
  depth.depth_data_nbytes =msg->depth.depth_data_nbytes;
  depth.depth_data =msg->depth.depth_data;
  depth.compression = msg->depth.compression;
  depth.uncompressed_size = 480*640*2;//msg->depth.uncompressed_size; // the original value was not properly filled in
  //lcm_->publish("KINECT_DEPTH_ONLY", &depth);

  openni::disparity_msg_t disp;
  disp.disparity_data_nbytes = 0;

  openni::frame_msg_t out;
  out.timestamp = msg->timestamp;
  out.image = img;
  out.depth = depth;
  out.disparity = disp;
  lcm_->publish("KINECT_DATA", &out);
}


int main(int argc, char ** argv) {

  lcm::LCM* lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  rgb_tool app(lcm);
  std::cout << "Ready image tool" << std::endl << "============================" << std::endl;
  while(0 == lcm->handle());
  return 0;
}
