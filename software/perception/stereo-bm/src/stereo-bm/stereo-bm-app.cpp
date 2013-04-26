#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "stereo-bm.hpp"
#include <ConciseArgs>

using namespace cv;
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, float scale_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   

    std::string image_channel_;
    StereoB*  stereob_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, float scale):
    lcm_(lcm_), image_channel_(image_channel_){

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);
  stereob_ = new StereoB(lcm_);
  stereob_->setScale(scale);
}



void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  cv::Mat left_img, right_img;

  int w = msg->width;
  int h = msg->height/2;
  
  if (left_img.empty() || left_img.rows != h || left_img.cols != w)
        left_img.create(h, w, CV_8UC1);
  if (right_img.empty() || right_img.rows != h || right_img.cols != w)
        right_img.create(h, w, CV_8UC1);

  memcpy(left_img.data,  msg->data.data() , msg->size/2);
  memcpy(right_img.data,  msg->data.data() + msg->size/2 , msg->size/2);

  stereob_->doStereoB(left_img, right_img);
  stereob_->sendRangeImage(msg->utime);
/*
  // extract image data
  switch (msg->pixelformat) {
//    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      ///  memcpy(img.data, msg->data, sizeof(uint8_t) * w * h * 3);
      std::copy(msg->data             , msg->data+msg->size/2 , left_img.data);
      std::copy(msg->data+msg->size/2 , msg->data+msg->size   , right_img.data);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      printf("jpegs not supported yet\n");
      exit(-1);
      // for some reason msg->row_stride is 0, so we use msg->width instead.
      //jpeg_decompress_8u_gray(msg->data,
      //                        msg->size,
      //                        img.data,
      //                        msg->width,
      //                        msg->height,
      //                        msg->width);
      break;
    default:
      fprintf(stderr, "Unrecognized image format\n");
      break;
  }
*/
}



int main(int argc, char ** argv) {
  string channel = "CAMERA";
  float scale = 0.25;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel","channel");
  opt.add(scale, "s", "scale","scale");
  opt.parse();
  std::cout << "channel: " << channel << "\n";  
  std::cout << "scale: " << scale << "\n";  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,channel,scale);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
