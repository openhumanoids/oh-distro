// Stereo block matcher using opencv

#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/bot2_param.h>
#include <signal.h>
#include <math.h>

// libbot/lcm includes
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_lcmgl_client/lcmgl.h>

// Opencv 
#include <opencv2/opencv.hpp>

#include <jpeg-utils/jpeg-utils.h>
#include <jpeg-utils/jpeg-utils-ijg.h>
#include <zlib.h>
#include <ConciseArgs>

using namespace std;

struct CameraParams { 
    cv::Mat_<double> K; 
    cv::Mat_<double> D; 

    int width, height;
    float fx, fy, cx, cy, k1, k2, k3, p1, p2;
    CameraParams () {}
    cv::Mat_<double> getK() { 
        K = cv::Mat_<double>::zeros(3,3);
        K(0,0) = fx, K(1,1) = fy; 
        K(0,2) = cx, K(1,2) = cy;
        K(2,2) = 1;
        return K;
    }
    cv::Mat_<double> getD() { 
        D = cv::Mat_<double>::zeros(1,5);
        D(0,0) = k1, D(0,1) = k2; 
        D(0,2) = p1, D(0,2) = p2;
        D(0,4) = k3;
        return D;
    }
};
CameraParams left_camera_params, right_camera_params;

struct state_t { 
    lcm_t *lcm;
    GMainLoop *mainloop;
    BotParam   *param;
    BotFrames *frames;
    bot_lcmgl_t *lcmgl;
};
state_t* state = NULL;

// Stereo related variables
cv::Mat left_stereo, right_stereo;
std::string channel_left, channel_right;

// Enums
enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };

// Parameters
int alg = STEREO_HH;
int SADWindowSize = 0, numberOfDisparities = 0;
bool no_display = false;
float scale = .25f;

cv::StereoBM bm;
cv::StereoSGBM sgbm;
cv::StereoVar var;

void
publish_opencv_image(lcm_t* lcm, const char* str, const cv::Mat& _img, double utime = 0) { 

    cv::Mat img; 
    int ch = _img.channels();
    if (ch == 3) 
        cv::cvtColor(_img, img, CV_BGR2RGB);
    else 
        img = _img.clone();

  bot_core_image_t bot_img;
  bot_img.utime = utime; 
  bot_img.width = img.cols; 
  bot_img.height = img.rows;
  bot_img.row_stride = img.cols * ch; 
  bot_img.pixelformat = (ch==3) ? BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB : BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  bot_img.size = img.rows * img.cols * ch;
  bot_img.data = (uint8_t*) img.data;
  bot_img.nmetadata = 0;
  bot_img.metadata = NULL;

  bot_core_image_t_publish(lcm, str, &bot_img);

  return;
}

void
decode_image(const bot_core_image_t * msg, cv::Mat& img)
{
    if (img.empty() || img.rows != msg->height || img.cols != msg->width)
        img.create(msg->height, msg->width, CV_8UC3);

  // extract image data
  // TODO add support for raw RGB
  switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height * 3);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // for some reason msg->row_stride is 0, so we use msg->width instead.
      jpeg_decompress_8u_gray(msg->data,
                              msg->size,
                              img.data,
                              msg->width,
                              msg->height,
                              msg->width);
      break;
    default:
      fprintf(stderr, "Unrecognized image format\n");
      break;
  }
}


void on_image_frame(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_image_t *msg, void *user_data)
{
    // Comp *self = (Comp*) user_data;
    state_t* self = (state_t*) user_data;
  
    if (strcmp(channel_left.c_str(), channel) == 0) { 
        decode_image(msg, left_stereo);
        if( scale != 1.f ) {
            int method = scale < 1 ? cv::INTER_AREA : cv::INTER_CUBIC;
            cv::resize(left_stereo, left_stereo, cv::Size(), scale, scale, method);
        }
        cv::cvtColor(left_stereo, left_stereo, CV_RGB2BGR);
    }  else if (strcmp(channel_right.c_str(), channel) == 0) { 
        decode_image(msg, right_stereo);
        if( scale != 1.f ) {
            int method = scale < 1 ? cv::INTER_AREA : cv::INTER_CUBIC;
            cv::resize(right_stereo, right_stereo, cv::Size(), scale, scale, method);
        }
        cv::cvtColor(right_stereo, right_stereo, CV_RGB2BGR);
    }
  else  
      std::cerr << "Unrecognized channel name" << std::endl;

  if (left_stereo.empty() || right_stereo.empty() || 
      (left_stereo.size() != right_stereo.size()))
      return;

  std::cerr << "RECEIVED both LEFT & RIGHT" << std::endl;

  // cv::Mat left, right; 
  // cv::cvtColor(left_stereo, left_stereo, CV_BGR2GRAY);
  // cv::cvtColor(right_stereo, right_stereo, CV_BGR2GRAY);  

  cv::Mat_<double> R = cv::Mat_<double>::eye(3,3);
  cv::Mat_<double> t = cv::Mat_<double>::zeros(3,1);
  t(1,0) = .7f; 

  cv::Rect roi_left, roi_right;
  cv::Mat_<double> Rleft, Rright, Pleft, Pright, Q;

  cv::Mat_<double> Mleft = left_camera_params.getK(), Mright = right_camera_params.getK(); 
  cv::Mat_<double> Dleft = left_camera_params.getD(), Dright = right_camera_params.getD(); 
  if( scale != 1.f ) {
      Mleft(0,2) *= scale, Mleft(1,2) *= scale;
      Mright(0,2) *= scale, Mright(1,2) *= scale;
  }
  
  cv::stereoRectify(Mleft, Dleft, Mright, Dright, 
                    left_stereo.size(), R, t, Rleft, Rright, Pleft, Pright, Q, 0, 
                    -1, left_stereo.size(), &roi_left, &roi_right);

  cv::Mat map11, map12, map21, map22;
  initUndistortRectifyMap(Mleft, Dleft, Rleft, Pleft, left_stereo.size(), CV_16SC2, map11, map12);
  initUndistortRectifyMap(Mright, Dright, Rright, Pright, right_stereo.size(), CV_16SC2, map21, map22);

  cv::Mat img1r, img2r;
  remap(left_stereo, img1r, map11, map12, cv::INTER_LINEAR);
  remap(right_stereo, img2r, map21, map22, cv::INTER_LINEAR);

  cv::Mat left = left_stereo.clone();
  cv::Mat right = right_stereo.clone();

  publish_opencv_image(self->lcm, "CAMERALEFT_RECTIFIED", left, msg->utime);
  publish_opencv_image(self->lcm, "CAMERARIGHT_RECTIFIED", right, msg->utime);

  numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((left.size().width/8) + 15) & -16;

  // Block matching
  bm.state->roi1 = roi_left;
  bm.state->roi2 = roi_right;
  bm.state->preFilterCap = 31;
  bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
  bm.state->minDisparity = 0;
  bm.state->numberOfDisparities = numberOfDisparities;
  bm.state->textureThreshold = 10;
  bm.state->uniquenessRatio = 15;
  bm.state->speckleWindowSize = 100;
  bm.state->speckleRange = 32;
  bm.state->disp12MaxDiff = 1;

  sgbm.preFilterCap = 63;
  sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

  // Channels
  int cn = left.channels();

  sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.minDisparity = 0;
  sgbm.numberOfDisparities = numberOfDisparities;
  sgbm.uniquenessRatio = 10;
  sgbm.speckleWindowSize = bm.state->speckleWindowSize;
  sgbm.speckleRange = bm.state->speckleRange;
  sgbm.disp12MaxDiff = 1;
  sgbm.fullDP = alg == STEREO_HH;

  var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
  var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
  var.nIt = 25;
  var.minDisp = -numberOfDisparities;
  var.maxDisp = 0;
  var.poly_n = 3;
  var.poly_sigma = 0.0;
  var.fi = 15.0f;
  var.lambda = 0.03f;
  var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
  var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
  var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

  cv::Mat disp, disp8;
  //Mat img1p, img2p, dispp;
  //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
  //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
  
  int64 tm = cv::getTickCount();
  if( alg == STEREO_BM )
      bm(left, right, disp);
  else if( alg == STEREO_VAR ) 
      var(left, right, disp);
  else if( alg == STEREO_SGBM || alg == STEREO_HH )
      sgbm(left, right, disp);
  tm = cv::getTickCount() - tm;
  printf("Time elapsed: %f ms\n", tm*1000/cv::getTickFrequency());

  if( scale != 1.f ) {
      int method =scale < 1 ? cv::INTER_AREA : cv::INTER_CUBIC;
      cv::resize(disp, disp, cv::Size(), 1.f/scale, 1.f/scale, method);
  }

  //disp = dispp.colRange(numberOfDisparities, img1p.cols);
  if( alg != STEREO_VAR )
      disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
  else
      disp.convertTo(disp8, CV_8U);

  if (disp.empty())
      return;
  
  publish_opencv_image(self->lcm, "CAMERADEPTH", disp8);

  return;
}

int main(int argc, char ** argv) {
    channel_left = std::string("CAMERALEFT");
    channel_right = std::string("CAMERARIGHT");
    ConciseArgs opt(argc, (char**)argv);
    opt.add(scale, "s", "scale", "Block-Matching Scale");
    opt.add(channel_left, "cl", "channel-left", "Left camera subscribe channel");
    opt.add(channel_right, "cr", "channel-right", "Right camera subscribe channel");
    opt.parse();
    std::cerr << "===========  Stereo Block Matcher ============" << std::endl;
    std::cerr << "=> LEFT Channel: " << channel_left << std::endl
              << "=> RIGHT Channel: " << channel_right << std::endl
              << "=> SCALE : " << scale << std::endl;
    std::cerr << "**********************************************" << std::endl;

    state_t *state = new state_t;

    // Param server, botframes
    state->lcm =  bot_lcm_get_global(NULL);
    state->param = bot_param_new_from_server(state->lcm, 1);
    state->frames = bot_frames_get_global (state->lcm, state->param);
    state->lcmgl = bot_lcmgl_init(state->lcm,"stereo-bm");


    std::string key_prefix_str = "cameras."+channel_left+".intrinsic_cal";
    left_camera_params.width = bot_param_get_int_or_fail(state->param, (key_prefix_str+".width").c_str());
    left_camera_params.height = bot_param_get_int_or_fail(state->param,(key_prefix_str+".height").c_str());
    left_camera_params.fx = bot_param_get_double_or_fail(state->param, (key_prefix_str+".fx").c_str());
    left_camera_params.fy = bot_param_get_double_or_fail(state->param, (key_prefix_str+".fy").c_str());
    left_camera_params.cx = bot_param_get_double_or_fail(state->param, (key_prefix_str+".cx").c_str());
    left_camera_params.cy = bot_param_get_double_or_fail(state->param, (key_prefix_str+".cy").c_str());
    left_camera_params.k1 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k1").c_str());
    left_camera_params.k2 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k2").c_str());
    left_camera_params.k3 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k3").c_str());
    left_camera_params.p1 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".p1").c_str());
    left_camera_params.p2 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".p2").c_str());

    key_prefix_str = "cameras."+channel_right+".intrinsic_cal";
    right_camera_params.width = bot_param_get_int_or_fail(state->param, (key_prefix_str+".width").c_str());
    right_camera_params.height = bot_param_get_int_or_fail(state->param,(key_prefix_str+".height").c_str());
    right_camera_params.fx = bot_param_get_double_or_fail(state->param, (key_prefix_str+".fx").c_str());
    right_camera_params.fy = bot_param_get_double_or_fail(state->param, (key_prefix_str+".fy").c_str());
    right_camera_params.cx = bot_param_get_double_or_fail(state->param, (key_prefix_str+".cx").c_str());
    right_camera_params.cy = bot_param_get_double_or_fail(state->param, (key_prefix_str+".cy").c_str());
    right_camera_params.k1 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k1").c_str());
    right_camera_params.k2 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k2").c_str());
    right_camera_params.k3 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".k3").c_str());
    right_camera_params.p1 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".p1").c_str());
    right_camera_params.p2 = bot_param_get_double_or_fail(state->param, (key_prefix_str+".p2").c_str());

    state->mainloop = g_main_loop_new( NULL, FALSE );  

    if (!state->mainloop) {
        printf("Couldn't create main loop\n");
        return -1;
    }

    bot_core_image_t_subscription_t * sub_left =  bot_core_image_t_subscribe(state->lcm, channel_left.c_str(), 
                                                                             on_image_frame, state);
    bot_core_image_t_subscription_t * sub_right =  bot_core_image_t_subscribe(state->lcm, channel_right.c_str(), 
                                                                              on_image_frame, state);

    //add lcm to mainloop 
    bot_glib_mainloop_attach_lcm (state->lcm);

    //adding proper exiting 
    bot_signal_pipe_glib_quit_on_kill (state->mainloop);

    ///////////////////////////////////////////////
    g_main_loop_run(state->mainloop);
  
    bot_glib_mainloop_detach_lcm(state->lcm);

    return 0; 
}
