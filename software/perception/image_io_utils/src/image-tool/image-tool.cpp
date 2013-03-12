// 1024x544 from Gazebo
// reduced by 4 to 256x136 - 5KB per image @ 50% quality 
// (1024x544 per image @ 50% quality is 40KB for same image)
//
// Older Calculations:
// 320x240 image:
// 1.8MB per sec @ 8Hz uncompressed [230kB per image = 320w*240h*3stride]
// 145KB per sec @ 8Hz 94% quality jpeg [18KB per image]
// 50KB per sec @ 8Hz 50% quality jpeg [6.25KB per image]
// 21kB per sec @ 8Hz 10% quality jpeg [1.6KB per image = about 13kbits]
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

using namespace cv;
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
      int resize_, int jpeg_quality_, int mode , int downsample_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void triggerHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::data_request_t* msg);   

    void sendOutput();

    int jpeg_quality_;

    int width_;
    int height_;
    int counter_;
    int resize_;
    std::string image_channel_;

    int mode_;
    int downsample_;

    image_io_utils*  imgutils_;
  
    bot_core::image_t last_img_;  
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, 
           int resize_, int jpeg_quality_, int mode_, int downsample_):
    lcm_(lcm_), resize_(resize_), jpeg_quality_(jpeg_quality_), mode_(mode_),
    image_channel_(image_channel_), downsample_(downsample_){

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);
  lcm_->subscribe("TRIGGER_CAMERA",&Pass::triggerHandler,this);

  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_ );
  width_ = 1024;
  height_ = 544;
  counter_=0;
  last_img_.utime=0; // used to indicate no message recieved yet
}

void Pass::sendOutput(){
  if (last_img_.utime==0){     return;   } // if no msg recieved then ignore output command
    
  if (mode_==0){ // resize, jpeg and send 
    /// factor of 4: 1024x544 --> 256x136  
    // OPENCV HERE TO AVOID DEPENDENCY
    Mat src= Mat::zeros( last_img_.height,last_img_.width  ,CV_8UC3);
    src.data = last_img_.data.data();
    int resize_height = last_img_.height/resize_;
    int resize_width  = last_img_.width/resize_;
    Mat img = Mat::zeros( resize_height , resize_width ,CV_8UC3);
    cv::resize(src, img, img.size());  // Resize src to img size
    imgutils_->jpegImageThenSend(img.data, last_img_.utime, 
                resize_width, resize_height, jpeg_quality_, image_channel_);
  }else if(mode_==1){ // unzip and send
    uint8_t* buf = imgutils_->unzipImage( &(last_img_) );// , image_channel_);
    imgutils_->sendImage(buf, last_img_.utime, last_img_.width, 
                         last_img_.height, 1, string(image_channel_ + "_UNZIPPED")  );
  }
}

void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  
  if (width_ != msg->width){
    cout << "incoming width " << msg->width << " doesn't match assumed width " << width_ << "\n";
    cout << "returning cowardly\n";
    return;
  }

  last_img_= *msg;  
  if(downsample_==-1){   return;   }
  if (counter_% downsample_ ==0){ sendOutput();  }    
}

void Pass::triggerHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::data_request_t* msg){
  sendOutput();
}


int main(int argc, char ** argv) {
  cout << "============= QUICK MODES ===================\n";
  cout << "drc-image-tool  -m 0 -c CAMERALEFT\n";
  cout << "drc-image-tool  -m 1 -c CAMERALEFT_MASKZIPPED\n";
  cout << "=============================================\n";

  int jpeg_quality = 50;
  string channel = "CAMERALEFT";
  int resize = 4;
  int mode=0;
  int downsample = -1;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(jpeg_quality, "j", "jpeg_quality","jpeg_quality");
  opt.add(channel, "c", "channel","channel");
  opt.add(resize, "r", "resize","resize image by this factor");
  opt.add(mode, "m", "mode","0=rgbinJPEGREDUCEDOUT 1=zipinGRAYOUT");
  opt.add(downsample, "d", "downsample","Downsample Factor");
  opt.parse();
  std::cout << "jpeg_quality: " << jpeg_quality << "\n";  
  std::cout << "channel: " << channel << "\n";  
  std::cout << "resize: " << resize << "\n";
  std::cout << "mode: " << mode << "\n";    
  std::cout << "downsample: output 1 in every [" << downsample << "] frames\n";
  std::cout << "            (set as -1 to output none regularly)\n";    
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,channel, resize, jpeg_quality, mode, downsample);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
