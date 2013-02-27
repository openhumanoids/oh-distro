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

#include <jpeg-utils/jpeg-utils.h>
#include <jpeg-utils/jpeg-utils-ijg.h>
#include <zlib.h>
#include <ConciseArgs>

using namespace cv;

using namespace std;

typedef struct _Comp {
  lcm_t* subscribe_lcm;
  lcm_t* publish_lcm;
  uint8_t* image_buf;
  int image_buf_size;
  int jpeg_quality;

  int width;
  int height;
  int counter;
  int resize;
}Comp;


void on_image_frame(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_image_t *msg, void *user_data)
{
  Comp *self = (Comp*) user_data;
    
  /// factor of 4: 1024x544 --> 256x136
  Mat src= Mat::zeros( msg->height,msg->width  ,CV_8UC3);
  src.data = msg->data;
  int resize_height = msg->height/self->resize;
  int resize_width  = msg->width/self->resize;
  Mat img = Mat::zeros( resize_height , resize_width ,CV_8UC3);
  cv::resize(src, img, img.size());  // Resize src to img size
  int compressed_size =  resize_width*resize_height*3;//image_buf_size;
  int compression_status = jpeg_compress_8u_rgb (img.data, resize_width, resize_height, resize_width*3,
                                                     self->image_buf, &compressed_size, self->jpeg_quality);

  bot_core_image_t msgout_small;
  msgout_small.utime = msg->utime;
  msgout_small.width = resize_width;
  msgout_small.height = resize_height;
  msgout_small.row_stride = 3*resize_width;
  msgout_small.size = compressed_size;
  msgout_small.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
  msgout_small.data = self->image_buf;
  msgout_small.nmetadata =0;
  msgout_small.metadata = NULL;
  string channel_out_small = string(channel) + "_COMPRESSED";
  bot_core_image_t_publish(self->publish_lcm, channel_out_small.c_str(), &msgout_small);
  
  self->counter++;
  if (self->counter%30 ==0){
     cout << self->counter << " | " << msg->utime << "\n"; 
  }  
}

int main(int argc, char ** argv) {
  int jpeg_quality = 50;
  string channel = "CAMERALEFT";
  int resize = 4;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(jpeg_quality, "j", "jpeg_quality","jpeg_quality");
  opt.add(channel, "c", "channel","channel");
  opt.add(resize, "r", "resize","resize image by this factor");
  opt.parse();
  std::cout << "jpeg_quality: " << jpeg_quality << "\n";  
  std::cout << "channel: " << channel << "\n";  
  std::cout << "resize: " << resize << "\n";  
  
  
  //
  Comp *self = (Comp*) calloc (1, sizeof (Comp));
  self->width = 1024;
  self->height = 544;
  self->resize = resize;
  self->jpeg_quality = jpeg_quality;
  self->image_buf_size = self->width * self->height * 10;
  if (0 != posix_memalign((void**) &self->image_buf, 16, self->image_buf_size)) {
    fprintf(stderr, "Error allocating image buffer\n");
    return 1;
  }
  self->publish_lcm=lcm_create(NULL);
  self->subscribe_lcm = lcm_create(NULL);
  self->counter =0;

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
