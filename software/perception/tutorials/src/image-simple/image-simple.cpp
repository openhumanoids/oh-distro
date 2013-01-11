//
// TODO:
// Add resizing as an option
//
// 320x240 images from Gazebo:
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


#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;
typedef struct _Comp {
  lcm_t* subscribe_lcm;
  lcm_t* publish_lcm;
  int counter;
}Comp;


void on_image_frame(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_image_t *msg, void *user_data)
{
  Comp *self = (Comp*) user_data;
  cout << "got an image\n";  
  
  Mat img_lcm = Mat::zeros(msg->height,msg->width,CV_8UC3); // h,w
  img_lcm.data = msg->data;
  Mat img;
  cvtColor( img_lcm, img, CV_BGR2RGB);
  imwrite("test.png",img); 

  /// process img here ///

}

int main(int argc, char ** argv) {
 
  
  //
  Comp *self = (Comp*) calloc (1, sizeof (Comp));
  self->publish_lcm=lcm_create(NULL);
  self->subscribe_lcm = lcm_create(NULL);
  self->counter =0;

  bot_core_image_t_subscription_t * sub =  bot_core_image_t_subscribe(self->subscribe_lcm, "CAMERALEFT", on_image_frame, self);
  /*  if (skip_frames){ // toss all frames except most recent
   cout << "Skipping most recent frames\n";
    bot_core_image_t_subscription_set_queue_capacity(sub,1);
  }*/

  // go!
  while(1)
    lcm_handle(self->subscribe_lcm);
  return 0;
}
