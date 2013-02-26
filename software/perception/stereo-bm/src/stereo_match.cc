// Stereo block matcher using opencv

#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/bot2_param.h>
#include <signal.h>
#include <math.h>


#include <lcmtypes/multisense.h>


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

static const char* FILTER_NAME = "Stereo Block-Matcher";
static const char* PARAM_PRE_FILTER_SIZE = "Pre-Filter Size";
static const char* PARAM_PRE_FILTER_CAP = "Pre-Filter Cap";
static const char* PARAM_CORRELATION_WINDOW_SIZE = "Correlation Window Size";
static const char* PARAM_MIN_DISPARITY = "Min. Disparity";
static const char* PARAM_NO_DISPARITIES = "Num. of Disparities";
static const char* PARAM_DISPARITY_RANGE = "Disparity Range";
static const char* PARAM_UNIQUENESS_RATIO = "Uniqueness Ratio";
static const char* PARAM_TEXTURE_THRESHOLD = "Texture Threshold";
static const char* PARAM_SPECKLE_WINDOW_SIZE = "Speckle Window Size";
static const char* PARAM_SPECKLE_RANGE = "Speckle Range";


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
    // BotGtkParamWidget* pw;
};
state_t* state = NULL;

// Enums
enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };

// Parameters
int alg = STEREO_HH;
int SADWindowSize = 0, numberOfDisparities = 0;
bool no_display = false;

cv::StereoBM bm;
cv::StereoSGBM sgbm;
cv::StereoVar var;


// Stereo BM params
// Variable Parameters
std::string vCHANNEL_LEFT, vCHANNEL_RIGHT;
float vSCALE = .25f;
bool vDEBUG = false;

int vPRE_FILTER_SIZE = 9; // 5-255
int vPRE_FILTER_CAP = 63; // 1-63
int vCORRELATION_WINDOW_SIZE = 3; // 5-255
int vMIN_DISPARITY = 0; // (-128)-128
// int vNO_DISPARITIES; // 
int vDISPARITY_RANGE = 64; // 32-128
int vUNIQUENESS_RATIO = 15; // 0-100
int vTEXTURE_THRESHOLD = 10; // 0-100
int vSPECKLE_WINDOW_SIZE = 100; // 0-1000
int vSPECKLE_RANGE = 4; // 0-31


void
publish_opencv_image(lcm_t* lcm, const char* str, const cv::Mat& _img, double utime = 0) { 

    cv::Mat img; 
    int ch = _img.channels();
    if (ch == 3) 
        cv::cvtColor(_img, img, CV_BGR2RGB);
    else 
        img = _img.clone();

    // std::cerr << str << " " << img.type() << " " << CV_16S << std::endl;

    bot_core_image_t bot_img;
    bot_img.utime = utime; 
    bot_img.width = img.cols; 
    bot_img.height = img.rows;
    bot_img.row_stride = img.cols * ch; 
    if ((ch == 1) && (img.type() == CV_16S)) {
        bot_img.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16;
        bot_img.row_stride = img.cols * ch * sizeof(uint16_t); 
        bot_img.size = img.rows * img.cols * ch * sizeof(uint16_t);
    } else { 
        bot_img.pixelformat = (ch==3) ? BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB : BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
        bot_img.row_stride = img.cols * ch; 
        bot_img.size = img.rows * img.cols * ch;
    }
    bot_img.data = (uint8_t*) img.data;
    bot_img.nmetadata = 0;
    bot_img.metadata = NULL;

    bot_core_image_t_publish(lcm, str, &bot_img);

  return;
}



// Publish both left and disparity in one image type:
cv::Mat temp_img;
void
publish_opencv_image_multisense(lcm_t* lcm, const char* str, const cv::Mat& _disp, const cv::Mat& _left,  double utime ,
  const bot_core_image_t * msg) { 
  int h = _disp.rows;
  int w = _disp.cols;
  
  //////// RIGHT IMAGE /////////////////////////////////////////////  
  int n_bytes=4; // 4 bytes per value
  int isize = n_bytes*w*h;
  bot_core_image_t disp_msg_out;
  disp_msg_out.utime = utime; 
  disp_msg_out.width = w; 
  disp_msg_out.height = h;
  disp_msg_out.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32;
  disp_msg_out.row_stride = n_bytes*w;
  disp_msg_out.size = isize;
  disp_msg_out.data = (uint8_t*) _disp.data;
  disp_msg_out.nmetadata = 0;
  disp_msg_out.metadata = NULL;

  //////// LEFT IMAGE //////////////////////////////////////////////
  if (temp_img.empty() || temp_img.rows != h || temp_img.cols != w)
        temp_img.create(h, w, CV_8UC1);    
  bot_core_image_t left_msg_out;
  left_msg_out.utime = utime; 
  left_msg_out.width = w;
  left_msg_out.height =h;
  left_msg_out.row_stride = w; 
  left_msg_out.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  left_msg_out.size = w*h;
  std::copy(msg->data             , msg->data+msg->size/2 , temp_img.data);
  left_msg_out.data = (uint8_t*) temp_img.data;
  left_msg_out.nmetadata = 0;
  left_msg_out.metadata = NULL;

  //////// BOTH IMAGES /////////////////////////////////////////////
  multisense_images_t images;
  images.utime = utime;
  images.n_images=2;
  int16_t im_types[ 2 ];
  im_types[0] = MULTISENSE_IMAGES_T_LEFT;
  im_types[1] = MULTISENSE_IMAGES_T_DISPARITY;
  bot_core_image_t im_list[ 2 ];
  im_list[0] = left_msg_out;
  im_list[1] = disp_msg_out;
  images.image_types = im_types;
  images.images = im_list;
  multisense_images_t_publish(lcm, "MULTISENSE_LD", &images);        
  return;
}


void
decode_image(const bot_core_image_t * msg, cv::Mat& left_img, cv::Mat& right_img)
{
  int w = msg->width;
  int h = msg->height/2;
  
  if (left_img.empty() || left_img.rows != h || left_img.cols != w)
        left_img.create(h, w, CV_8UC1);
  if (right_img.empty() || right_img.rows != h || right_img.cols != w)
        right_img.create(h, w, CV_8UC1);

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
}


// Stereo related variables
cv::Mat left_stereo, right_stereo;

void on_image_frame(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_image_t *msg, void *user_data)
{
  state_t* self = (state_t*) user_data;
  decode_image(msg, left_stereo, right_stereo);
  
  if( vSCALE != 1.f ) {
      int method = vSCALE < 1 ? cv::INTER_AREA : cv::INTER_CUBIC;
      cv::resize(left_stereo, left_stereo, cv::Size(), vSCALE, vSCALE, method);
  }
  //cv::cvtColor(left_stereo, left_stereo, CV_RGB2BGR);

  if( vSCALE != 1.f ) {
      int method = vSCALE < 1 ? cv::INTER_AREA : cv::INTER_CUBIC;
      cv::resize(right_stereo, right_stereo, cv::Size(), vSCALE, vSCALE, method);
  }
  //cv::cvtColor(right_stereo, right_stereo, CV_RGB2BGR);

  
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
  if( vSCALE != 1.f ) {
      Mleft(0,2) *= vSCALE, Mleft(1,2) *= vSCALE;
      Mright(0,2) *= vSCALE, Mright(1,2) *= vSCALE;
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

  // publish_opencv_image(self->lcm, "CAMERALEFT_RECTIFIED", left, msg->utime);
  // publish_opencv_image(self->lcm, "CAMERARIGHT_RECTIFIED", right, msg->utime);

  numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((left.size().width/8) + 15) & -16;

  // Block matching
  bm.state->roi1 = roi_left;
  bm.state->roi2 = roi_right;
  bm.state->preFilterCap = vPRE_FILTER_CAP;
  bm.state->SADWindowSize = vCORRELATION_WINDOW_SIZE;
  bm.state->minDisparity = vMIN_DISPARITY;
  bm.state->numberOfDisparities = numberOfDisparities;
  bm.state->textureThreshold = vTEXTURE_THRESHOLD;
  bm.state->uniquenessRatio = vUNIQUENESS_RATIO;
  bm.state->speckleWindowSize = vSPECKLE_WINDOW_SIZE;
  bm.state->speckleRange = vSPECKLE_RANGE;
  bm.state->disp12MaxDiff = 1;

  sgbm.preFilterCap = vPRE_FILTER_CAP; // 63
  sgbm.SADWindowSize = vCORRELATION_WINDOW_SIZE; // 3

  // Channels
  int cn = left.channels();

  sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.minDisparity = vMIN_DISPARITY;
  sgbm.numberOfDisparities = numberOfDisparities;
  sgbm.uniquenessRatio = vUNIQUENESS_RATIO;
  sgbm.speckleWindowSize = vSPECKLE_WINDOW_SIZE;
  sgbm.speckleRange = vSPECKLE_RANGE;
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

  if( vSCALE != 1.f ) {
      int method =vSCALE < 1 ? cv::INTER_AREA : cv::INTER_CUBIC;
      cv::resize(disp, disp, cv::Size(), 1.f/vSCALE, 1.f/vSCALE, method);
  }

  //disp = dispp.colRange(numberOfDisparities, img1p.cols);
  if( alg != STEREO_VAR )
      disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
  else
      disp.convertTo(disp8, CV_8U);

  if (disp.empty())
      return;
  
  if (vDEBUG)
      publish_opencv_image(self->lcm, "CAMERADEPTH8", disp8, msg->utime);
  
  publish_opencv_image_multisense(self->lcm, "CAMERADEPTH", disp, left_stereo, msg->utime, msg);

  
  std::stringstream disparity_fname;
  disparity_fname << "disparity_lcm_" << msg->utime << ".png";
  imwrite(disparity_fname.str(),disp); 
  printf("finished write\n");
  
  return;
}

int main(int argc, char ** argv) {
    vCHANNEL_LEFT = std::string("CAMERALEFT");
    vCHANNEL_RIGHT = std::string("CAMERARIGHT");
    ConciseArgs opt(argc, (char**)argv);
    opt.add(vSCALE, "s", "scale", "Block-Matching Scale");
    opt.add(vCHANNEL_LEFT, "cl", "channel-left", "Left camera subscribe channel");
    opt.add(vCHANNEL_RIGHT, "cr", "channel-right", "Right camera subscribe channel");

    // stereo bm params
    opt.add(vPRE_FILTER_SIZE, "pfs", "pre-filter-size", PARAM_PRE_FILTER_SIZE);
    opt.add(vPRE_FILTER_CAP, "pfc", "pre-filter-cap", PARAM_PRE_FILTER_CAP);
    opt.add(vCORRELATION_WINDOW_SIZE, "w", "correlation-window-size", PARAM_CORRELATION_WINDOW_SIZE);
    opt.add(vMIN_DISPARITY, "md", "min-disparity", PARAM_MIN_DISPARITY);
    opt.add(vDISPARITY_RANGE, "dr", "disparity-range", PARAM_DISPARITY_RANGE);
    opt.add(vUNIQUENESS_RATIO, "ur", "uniqueness-ratio", PARAM_UNIQUENESS_RATIO);
    opt.add(vTEXTURE_THRESHOLD, "tt", "texture-threshold", PARAM_TEXTURE_THRESHOLD);
    opt.add(vSPECKLE_WINDOW_SIZE, "sws", "speckle-window-size", PARAM_SPECKLE_WINDOW_SIZE);
    opt.add(vSPECKLE_RANGE, "sr", "speckle-range", PARAM_SPECKLE_RANGE);

    opt.add(vDEBUG, "d", "debug", "Debug mode");
    opt.parse();

    std::cerr << "===========  Stereo Block Matcher ============" << std::endl;
    std::cerr << "=> LEFT Channel: " << vCHANNEL_LEFT << std::endl
              << "=> RIGHT Channel: " << vCHANNEL_RIGHT << std::endl
              << "=> SCALE : " << vSCALE << std::endl
              << "=> DEBUG : " << vDEBUG << std::endl;
    std::cerr << "**********************************************" << std::endl;

    state_t *state = new state_t;

    // Param server, botframes
    state->lcm =  bot_lcm_get_global(NULL);
    state->param = bot_param_new_from_server(state->lcm, 1);
    state->frames = bot_frames_get_global (state->lcm, state->param);
    state->lcmgl = bot_lcmgl_init(state->lcm,"stereo-bm");
    // state->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());

    std::string key_prefix_str = "cameras."+vCHANNEL_LEFT+".intrinsic_cal";
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

    key_prefix_str = "cameras."+vCHANNEL_RIGHT+".intrinsic_cal";
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

    bot_core_image_t_subscription_t * sub =  bot_core_image_t_subscribe(state->lcm, "CAMERA", 
                                                                             on_image_frame, state);
//    bot_core_image_t_subscription_t * sub_right =  bot_core_image_t_subscribe(state->lcm, vCHANNEL_RIGHT.c_str(), 
//                                                                              on_image_frame, state);

    //add lcm to mainloop 
    bot_glib_mainloop_attach_lcm (state->lcm);

    //adding proper exiting 
    bot_signal_pipe_glib_quit_on_kill (state->mainloop);

    ///////////////////////////////////////////////
    g_main_loop_run(state->mainloop);
  
    bot_glib_mainloop_detach_lcm(state->lcm);

    return 0; 
}
