// mfallon sept 2012

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/bot_param_update_t.h>
#include "lcmgl_example.hpp"

using namespace std;
using namespace cv;

lcmgl_example::lcmgl_example(lcm_t* publish_lcm,lcm_t* subscribe_lcm):
          publish_lcm_(publish_lcm),
          subscribe_lcm_(subscribe_lcm){
  lcmgl_ = bot_lcmgl_init(publish_lcm, "lcmgl-example");
}

int lcmgl_example::do_some_lcmgl(int argc, char **argv){
  // Read and renderer an image:
  // 1a. Read image and convert to RGB: 
  Mat src,src_rgb;
  src = imread("example.png",1);
  cvtColor(src,src_rgb,CV_BGR2RGB); // opencv stores data as BGR
  // 1b. Convert image into a GL texture:
  const uint8_t* target_gray = src_rgb.data;
  int height = src.rows;
  int width = src.cols;
  int row_stride = 3*width;//msg->row_stride;
  cout << height << " " << width << " " <<  row_stride << "\n";
  int gray_texid = bot_lcmgl_texture2d(lcmgl_, target_gray, width, height,
                                       row_stride, BOT_LCMGL_RGB, // grey:BOT_LCMGL_LUMINANCE,
                                       BOT_LCMGL_UNSIGNED_BYTE,
                                       BOT_LCMGL_COMPRESS_NONE);
  // 1c. Publish the LCMGL:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_color3f(lcmgl_, 1, 1, 1);
  bot_lcmgl_translated(lcmgl_, 20, 10, 0); // example offset
  bot_lcmgl_texture_draw_quad(lcmgl_, gray_texid,
      0     , 0      , 0   ,
      0     , height , 0   ,
      width , height , 0   ,   ///... where to put the image
      width , 0      , 0);  
  bot_lcmgl_pop_matrix(lcmgl_);  

  // 2. Create and publish 100 points to LCMGL:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_translated(lcmgl_, 0, 70, 0);  // example offset
  bot_lcmgl_point_size(lcmgl_, 1.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue
  for (size_t i=0; i < 100; ++i) {
    bot_lcmgl_vertex3f(lcmgl_, i,  i, i); // 100 points in line
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

  // 2. Create and publish 3 connected lines to LCMGL:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_translated(lcmgl_, 0, 40, 0);  // example offset
  bot_lcmgl_point_size(lcmgl_, 1.5f);
  bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);  // render as points
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); // Red
  bot_lcmgl_vertex3f(lcmgl_, 1,  2, 3);
  bot_lcmgl_vertex3f(lcmgl_, 6,  9, 10);
  bot_lcmgl_vertex3f(lcmgl_, -5,  2, 12);
  bot_lcmgl_vertex3f(lcmgl_, 5,  2, -12);
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);
  bot_lcmgl_switch_buffer(lcmgl_);  


  return 1;
}

int main(int argc, char ** argv) {
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  lcmgl_example app(lcm,lcm);
  int status =  app.do_some_lcmgl(argc, argv);
  return 0;
}
