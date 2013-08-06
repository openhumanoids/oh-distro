// TODO: read config from paramserver

#include <iostream>
#include <vector>
#include "multisense_renderer.h"

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/multisense_images_t.h>

#define PARAM_HISTORY_FREQUENCY "Scan Hist. Freq."
#define MAX_REFERSH_RATE_USEC 30000 //not sure whether we really need this part

const char* PARAM_POINT_WIDTH_POINTS = "Point width";
const double PARAM_POINT_WIDTH_POINTS_MIN = 1;
const double PARAM_POINT_WIDTH_POINTS_MAX = 30;
const double PARAM_POINT_WIDTH_POINTS_DELTA = 1;
const double PARAM_POINT_WIDTH_POINTS_DEFAULT = 2; // previous fixed value


#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

cv::Mat_<double> Q_(4,4,0.0);

typedef struct _MultisenseRenderer {
  BotRenderer renderer;
  BotGtkParamWidget *pw;
  BotViewer   *viewer;
  lcm_t   *lcm;
  BotFrames *frames;
  char * camera_frame;
  char * camera_channel;
  char * renderer_name;

  multisense_images_t* msg;

  int64_t last_utime;

  int width;
  int height;

  int need_to_recompute_frame_data;

  //fovis::StereoCalibration kcal;

  std::vector<float> disparity_buff_;
  std::vector<cv::Vec3f> points_buff_;
  
  double camera_to_local_m_opengl[16];
  // camera calib:
  double baseline;
  double left_cx;
  double right_cx;
  double right_cy;
  double right_fx;
  double right_fy;

  
  double param_point_width;
  
} MultisenseRenderer;

static void
recompute_frame_data(MultisenseRenderer* self)
{
  if(!self->msg) {
    self->need_to_recompute_frame_data = 0;
    return;
  }

  int h = self->msg->images[0].height;
  int w = self->msg->images[0].width;

  // Allocate buffer for reprojection output
  self->points_buff_.resize(h * w);
  cv::Mat_<cv::Vec3f> points(h, w, &(self->points_buff_[0]));
  
  //std::cout << (int) self->msg->image_types[1]  << " types are\n";
  if( self->msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY){
    // Convert Carnegie disparity format into floating point disparity. Store in local buffer
    cv::Mat disparity_orig_temp = cv::Mat::zeros(h,w,CV_16UC1); // h,w
    disparity_orig_temp.data = self->msg->images[1].data;  
    cv::Mat_<uint16_t> disparity_orig(h, w);
    disparity_orig = disparity_orig_temp;

    self->disparity_buff_.resize(h * w);
    cv::Mat_<float> disparity(h, w, &(self->disparity_buff_[0]));
    disparity = disparity_orig / 16.0;

    // Do the reprojection in open space
    static const bool handle_missing_values = true;
    cv::reprojectImageTo3D(disparity, points, Q_, handle_missing_values);
  }else if( self->msg->image_types[1] == MULTISENSE_IMAGES_T_DEPTH_MM){
    std::cout << "msg got\n";
    uint16_t* depths =(uint16_t*)   self->msg->images[1].data;
    for(int v=0; v<h; v++) {
      for(int u=0; u<w; u++) {
        int pixel = v*w +u;
        float z = (float) depths[pixel] /1000.0;
        points(v,u)[2] =z;
        points(v,u)[0] = ( z * (u  - self->right_cx))/ self->right_fx ; // these params might not be right TODO: check
        points(v,u)[1] = ( z * (v  - self->right_cy))/ self->right_fy ;
      }
    }
  }

  
  if (self->frames!=NULL && bot_frames_have_trans(self->frames,self->camera_frame,bot_frames_get_root_name(self->frames))){
    double camera_to_local_m[16];
    bot_frames_get_trans_mat_4x4_with_utime(self->frames,self->camera_frame,bot_frames_get_root_name(self->frames),
    self->msg->utime, camera_to_local_m);
    // opengl expects column-major matrices
    bot_matrix_transpose_4x4d(camera_to_local_m, self->camera_to_local_m_opengl);  
  }
  
  self->need_to_recompute_frame_data=0;
}

static void 
on_camera_frame (const lcm_recv_buf_t *rbuf, const char *channel,
    const multisense_images_t *msg, void *user_data )
{
  MultisenseRenderer *self = (MultisenseRenderer*) user_data;

  static int64_t last_redraw_utime = 0;
  int64_t now = bot_timestamp_now(); 
  double hist_spc = bot_gtk_param_widget_get_double (self->pw, PARAM_HISTORY_FREQUENCY);

  if (abs (msg->utime - self->last_utime) > (int64_t)(1E6/hist_spc)) {
    self->last_utime = msg->utime;
    
    if(self->msg)
      multisense_images_t_destroy(self->msg);
    self->msg = multisense_images_t_copy(msg);
    self->need_to_recompute_frame_data = 1;
    
    if ((now - last_redraw_utime) > MAX_REFERSH_RATE_USEC) {
      bot_viewer_request_redraw( self->viewer );
      last_redraw_utime = now;
    }
    
    bot_viewer_request_redraw(self->viewer);
  }
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
  MultisenseRenderer *self = (MultisenseRenderer*) user;

  self->param_point_width = (double)bot_gtk_param_widget_get_double(self->pw, PARAM_POINT_WIDTH_POINTS);
  
  bot_viewer_request_redraw(self->viewer);
}

static inline void
_matrix_transpose_4x4d (const double m[16], double result[16])
{
  result[0] = m[0];
  result[1] = m[4];
  result[2] = m[8];
  result[3] = m[12];
  result[4] = m[1];
  result[5] = m[5];
  result[6] = m[9];
  result[7] = m[13];
  result[8] = m[2];
  result[9] = m[6];
  result[10] = m[10];
  result[11] = m[14];
  result[12] = m[3];
  result[13] = m[7];
  result[14] = m[11];
  result[15] = m[15];
}

bool IsFiniteNumber(double x) {
  return (x <= DBL_MAX && x >= -DBL_MAX); 
}  

static void _draw(BotViewer *viewer, BotRenderer *renderer){
  MultisenseRenderer *self = (MultisenseRenderer*) renderer->user;
  
  if(!self->msg)
    return;

  if(self->need_to_recompute_frame_data)
    recompute_frame_data(self);
  
  glPushMatrix();
  if (self->frames==NULL || !bot_frames_have_trans(self->frames,self->camera_frame,bot_frames_get_root_name(self->frames))){
    // rotate so that X is forward and Z is up
    glRotatef(-90, 0, 0, 1);
    glRotatef(-90, 1, 0, 0);
    
  }else{
    // I now assume that the recompute gets the pose and we dont move it then
    
    //project to current frame
    //double camera_to_local_m[16];
    //bot_frames_get_trans_mat_4x4_with_utime(self->frames,self->camera_frame,bot_frames_get_root_name(self->frames),
    //self->msg->utime, camera_to_local_m);
    //// opengl expects column-major matrices
    //double camera_to_local_m_opengl[16];
    //bot_matrix_transpose_4x4d(camera_to_local_m, camera_to_local_m_opengl);
    glMultMatrixd(self->camera_to_local_m_opengl);
  }
  
  cv::Mat_<cv::Vec3f> points( self->height ,self->width, &(self->points_buff_[0]));

  glPushMatrix();
  glEnable(GL_DEPTH_TEST);
  //glPointSize( 2.0f);
  glPointSize((GLfloat)self->param_point_width);
  
  glBegin(GL_POINTS);
  glColor3f(0, 0, 0);

  for(int v=0; v<self->height; v++) {
    for(int u=0; u<self->width; u++) {
      if ((IsFiniteNumber(points(v,u)[0])) && (IsFiniteNumber(points(v,u)[1])) ){
        uint8_t r, g, b;
        if( self->msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY){
          r = g = b = self->msg->images[0].data[ v*self->width + u ];
        }else if(self->msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB){
          int pixel = v*self->width + u;
          r = self->msg->images[0].data[ pixel*3 ];
          g = self->msg->images[0].data[ pixel*3 +1];
          b = self->msg->images[0].data[ pixel*3 +2];
        }
        glColor3f(r / 255.0, g / 255.0, b / 255.0);
        glVertex3f(points(v,u)[0], points(v,u)[1], points(v,u)[2]);
      }
    }
  }

  glEnd();
  glPopMatrix();
  glPopMatrix(); //camera_to_local
}

static void _free(BotRenderer *renderer){
  MultisenseRenderer *self = (MultisenseRenderer*) renderer;

  if(self->msg)
    multisense_images_t_destroy(self->msg);

  if(self->camera_frame)
    free(self->camera_frame);

  free(self);
}

inline void fillDefaultConfig(MultisenseRenderer *self){
  if (1==1){ /////// serial number 2 unit
    fprintf(stderr, "Using 1024x544 Dense Depth Config\n");
    self->width = 1024;
    self->height = 544;
    self->left_cx =  512.0;
    self->right_cx = 512.0;
    self->right_cy = 272.0;
    self->right_fx = 557.1886596679688;
    self->right_fy = 557.1886596679688;
    self->baseline = 0.0701564848423;  
  }else if (1==0){ /////// From the unit that Carnegie provided in Jan 2013
    fprintf(stderr, "Using 1024x544 Dense Depth Config [old]\n");
    self->width = 1024;
    self->height = 544;
    self->left_cx = 512.0;
    self->right_cx = 512.0;
    self->right_cy = 272.0;
    self->right_fx = 606.0344848632812;
    self->right_fy = 606.0344848632812;
    self->baseline = 0.07;
  }else if (1==0){
    fprintf(stderr, "Using 800x800 Dense Depth Config\n");
    self->width = 800;
    self->height = 800;
    self->left_cx = 400.5;
    self->right_cx = 400.5;
    self->right_cy = 400.5;
    self->right_fx = 476.7014;
    self->right_fy = 476.7014;
    self->baseline =0.07;
  }
}

void 
multisense_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm, 
            BotFrames * frames, const char * camera_frame, const char * camera_channel, BotParam *param)
{
  MultisenseRenderer *self = (MultisenseRenderer*) calloc(1, sizeof(MultisenseRenderer));
  self->need_to_recompute_frame_data = 0;

  std::string root_str = "cameras." + std::string(camera_channel); 
  if ( bot_param_has_key (param, root_str.c_str()) ){
    std::cout << "Using config for: " << camera_channel << "\n";
    std::string left_str = "cameras." + std::string(camera_channel) + ".left"; 
    self->width = bot_param_get_double_or_fail(param, (left_str+".width").c_str());
    self->height = bot_param_get_double_or_fail(param, (left_str+".height").c_str());
    self->left_cx = bot_param_get_double_or_fail(param, (left_str+".cx").c_str());
    std::string right_str = "cameras." + std::string(camera_channel) + ".right"; 
    self->right_fx = bot_param_get_double_or_fail(param, (right_str+".fx").c_str());
    self->right_fy = bot_param_get_double_or_fail(param, (right_str+".fy").c_str());
    self->right_cx = bot_param_get_double_or_fail(param, (right_str+".cx").c_str());
    self->right_cy = bot_param_get_double_or_fail(param, (right_str+".cy").c_str());
    std::string trans_str = "cameras." + std::string(camera_channel) + ".translation"; 
    self->baseline = fabs( bot_param_get_double_or_fail(param, trans_str.c_str() )  );
  }else{
    fillDefaultConfig(self);
  }
  
  Q_(0,0) = Q_(1,1) = 1.0;
  Q_(3,2) = 1.0 / self->baseline;
  Q_(0,3) = -self->right_cx;
  Q_(1,3) = -self->right_cy;
  Q_(2,3) =  self->right_fx;
  Q_(3,3) = (self->right_cx - self->left_cx ) / self->baseline;
  std::cout << Q_ << " is reprojectionMatrix\n";
  
  self->frames = frames;
  if ( camera_channel )
    self->camera_channel = strdup(camera_channel);
  if ( camera_frame )
    self->camera_frame = strdup(camera_frame);

  self->msg = NULL;
  BotRenderer *renderer = &self->renderer;
      
  self->lcm = lcm;
  self->viewer = viewer;
  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

  bot_gtk_param_widget_add_double(self->pw, PARAM_HISTORY_FREQUENCY, 
                  BOT_GTK_PARAM_WIDGET_SLIDER, 
                  0.1, 30, 0.1, 30.0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_POINT_WIDTH_POINTS,
                                  BOT_GTK_PARAM_WIDGET_SLIDER,
                                  PARAM_POINT_WIDTH_POINTS_MIN, PARAM_POINT_WIDTH_POINTS_MAX,
                                  PARAM_POINT_WIDTH_POINTS_DELTA,
                                  PARAM_POINT_WIDTH_POINTS_DEFAULT);  
  
  self->param_point_width = PARAM_POINT_WIDTH_POINTS_DEFAULT;
  

  renderer->draw = _draw;
  renderer->destroy = _free;
  std::string renderer_name ="Dense Depth: " + string(camera_channel);
  self->renderer_name = strdup( renderer_name.c_str() );
  
  renderer->name = self->renderer_name;//"Dense Depth";
  renderer->widget = GTK_WIDGET(self->pw);
  renderer->enabled = 1;
  renderer->user = self;

  g_signal_connect (G_OBJECT (self->pw), "changed",
            G_CALLBACK (on_param_widget_changed), self);

  multisense_images_t_subscribe(self->lcm, camera_channel, on_camera_frame, self);
  bot_viewer_add_renderer(viewer, renderer, priority);
}
