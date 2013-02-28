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
#include <getopt.h>
#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>
#include <signal.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression


#include <jpeg-utils/jpeg-utils.h>
#include <jpeg-utils/jpeg-utils-ijg.h>
#include <zlib.h>
#include <ConciseArgs>

using namespace cv;

using namespace std;

typedef struct _Comp {
  lcm_t* subscribe_lcm;
  lcm_t* publish_lcm;
  int jpeg_quality;

  int width;
  int height;
  int counter;
  int resize;

  int mode;

  image_io_utils*  imgutils_;
}Comp;


void on_image_frame(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_image_t *msg, void *user_data)
{
  Comp *self = (Comp*) user_data;
  if (self->width != msg->width){
    cout << "incoming width " << msg->width << " doesn't match assumed width " << self->width << "\n";
    cout << "returning cowardly\n";
    return;
  }
    
  if (self->mode==0){ // resize, jpeg and send 
    /// factor of 4: 1024x544 --> 256x136  
    // OPENCV HERE TO AVOID DEPENDENCY
    Mat src= Mat::zeros( msg->height,msg->width  ,CV_8UC3);
    src.data = msg->data;
    int resize_height = msg->height/self->resize;
    int resize_width  = msg->width/self->resize;
    Mat img = Mat::zeros( resize_height , resize_width ,CV_8UC3);
    cv::resize(src, img, img.size());  // Resize src to img size
    self->imgutils_->jpegImageThenSend(img.data, msg->utime, 
		resize_width, resize_height, self->jpeg_quality, channel);
  }else if(self->mode==1){ // unzip and send
    self->imgutils_->unzipImageThenSend( msg, channel);
  }

  self->counter++;
  if (self->counter%30 ==0){
     cout << self->counter << " | " << msg->utime << "\n"; 
  }  
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
  ConciseArgs opt(argc, (char**)argv);
  opt.add(jpeg_quality, "j", "jpeg_quality","jpeg_quality");
  opt.add(channel, "c", "channel","channel");
  opt.add(resize, "r", "resize","resize image by this factor");
  opt.add(mode, "m", "mode","0=rgbinJPEGREDUCEDOUT 1=zipinGRAYOUT");
  opt.parse();
  std::cout << "jpeg_quality: " << jpeg_quality << "\n";  
  std::cout << "channel: " << channel << "\n";  
  std::cout << "resize: " << resize << "\n";
  std::cout << "mode: " << mode << "\n";    
  
  /////////////////////////////////////
  Comp *self = (Comp*) calloc (1, sizeof (Comp));
  self->width = 1024;
  self->height = 544;
  self->resize = resize;
  self->jpeg_quality = jpeg_quality;
  self->mode = mode;

  self->publish_lcm=lcm_create(NULL);
  self->subscribe_lcm = lcm_create(NULL);
  self->counter =0;

  self->imgutils_ = new image_io_utils( self->publish_lcm, self->width, self->height);

  bot_core_image_t_subscription_t * sub =  bot_core_image_t_subscribe(self->subscribe_lcm, channel.c_str(), on_image_frame, self);
  /*  if (skip_frames){ // toss all frames except most recent
   cout << "Skipping most recent frames\n";
    bot_core_image_t_subscription_set_queue_capacity(sub,1);
  }*/

  // go!
  while(1)
    lcm_handle(self->subscribe_lcm);
  return 0;
}
