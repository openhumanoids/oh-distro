#include <stdio.h>
#include <unistd.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <getopt.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <vector>
#include <opencv2/nonfree/features2d.hpp>
#include <perception_opencv_utils/opencv_utils.hpp>
#include <image_io_utils/image_io_utils.hpp>
#include <bot_param/param_util.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>
#include <ConciseArgs>
#include <geom_utils/geometry.h>

#define MAX_GROUND_PROJECTION_DISTANCE 30//120
#define MAX_GROUND_PROJECTION_DISTANCE_SQ (MAX_GROUND_PROJECTION_DISTANCE*MAX_GROUND_PROJECTION_DISTANCE)

using namespace std;

struct RoadDetectorOptions { 
  bool vDEBUG;
  std::string vCHANNEL;
  RoadDetectorOptions () : 
    vCHANNEL(std::string("CAMERALEFT")), vDEBUG(false) {}
};

/*typedef struct _state_t state_t;

  struct _state_t {
  lcm_t *lcm;
  GMainLoop *mainloop;
  BotFrames *frames; 
  BotParam *param;
  GMutex *mutex;
  pthread_t  process_thread;
  int verbose;
  //do we add these to a queue??
  };*/

typedef struct _ImageVertex ImageVertex;
struct _ImageVertex {
  float tx;
  float ty;
  float vx;
  float vy;
  float vz;
};



class Terrain{
public:
  lcm_t *lcm;
  GMainLoop *mainloop;
  BotFrames *frames; 
  BotParam *param;
  bot_lcmgl_t * lcmgl_basic;
  bot_lcmgl_t * lcmgl;
  BotCamTrans *camtrans;
  char * coord_frame;

  int img_nvertices;
  ImageVertex *vertices;
  int *vert_indices;
  int n_vert_indices;
  point3d_t *pixel_rays_local;
  point2d_t *ground_projections_local;

  GMutex *mutex;
  RoadDetectorOptions options;
  pthread_t  process_thread;
  int verbose;
  //do we add these to a queue??
  vector<bot_core_image_t *> image_list;
  bot_core_image_t *last_image;
  //opencv_utils::DisplayWrapper dp;
  cv::Mat img, hsv_img;
  int64_t img_utime;
};

void project_to_ground(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat1b& mask, bot_lcmgl_t *lcmgl);

// Adapted from cv_line_template::convex_hull
void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                        int num_modalities, cv::Point offset, cv::Size size,
                        cv::Mat& dst)
{
  std::vector<cv::Point> points;
  for (int m = 0; m < num_modalities; ++m)
    {
      for (int i = 0; i < (int)templates[m].features.size(); ++i)
	{
	  cv::linemod::Feature f = templates[m].features[i];
	  points.push_back(cv::Point(f.x, f.y) + offset);
	}
    }

  std::vector<cv::Point> hull;
  cv::convexHull(points, hull);

  dst = cv::Mat::zeros(size, CV_8U);
  const int hull_count = (int)hull.size();
  const cv::Point* hull_pts = &hull[0];
  cv::fillPoly(dst, &hull_pts, &hull_count, 1, cv::Scalar(255));
}

void dilate_mask(cv::Mat1b& mask, cv::Mat1b& mask_new){

  //cvFindContours
  const int OFFSET = 3;
  //http://opencv.willowgarage.com/documentation/cpp/imgproc_image_filtering.html#dilate
  cv::dilate(mask, mask_new, cv::Mat(), cv::Point(-1,-1), OFFSET);
  /*cv::Mat1b mask_new;
    mask_new.create(mask.rows, mask.cols); 
    //mask_new.zeros(mask_new.rows, mask_new.cols);
    //std::cout << "Mask Type : " << mask.type() << " New Mask Type : " << mask_new.type() << std::endl;
    //fprintf(stderr, "Rows : %d - Columns : %d\n", mask.rows, mask.cols);

    bool road = false;
    int delta = 5;
  
    for(int x=0; x< mask.cols; x++){
    for(int y=0; y< mask.rows; y++){
    int min_x = fmax(0, x - delta);
    int max_x = fmin(mask.cols-1, x+delta);

    int min_y = fmax(0, y -delta);
    int max_y = fmin(mask.rows-1, y+delta);

    road = false;
    for(int dx = min_x; dx <=max_x; dx++){
    for(int dy = min_y; dy <=max_y; dy++){
    road = mask.at<bool>(dy, dx);
    if(road)
    break;
    }
    }
    //bool road = mask.at<bool>(y, x);
    mask_new.at<bool>(y,x) = road;//road;//bool(true);//road;
    }
    }
  */
  cv::Mat1b sm_display = mask_new.clone();
  cv::imshow("Dilated Mask", sm_display);
}

void find_contours(cv::Mat1b& mask, cv::Mat& filled_contour){

  cv::Size size(mask.cols, mask.rows);
  vector<vector<cv::Point> > contours;

  // find contours and store them all as a list
  cv::Mat1b mask_new = mask.clone();
  cv::findContours(mask_new, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  
  //fprintf(stderr, "No of countours : %d\n", contours.size());

  vector<cv::Point> approx;

  cv::Mat1b mask_count = cv::Mat1b::zeros(mask_new.rows, mask_new.cols);
  //mask_count.create(mask.rows, mask.cols);
  //mask_new.zeros(mask_new.rows, mask_new.cols);

  // test each contour
  int largest_countour_id = -1;
  int max_size = 0;

  for( size_t i = 0; i < contours.size(); i++){
    vector<cv::Point> c_contour = contours[i];
    if( max_size < c_contour.size()){
      max_size = c_contour.size();
      largest_countour_id = (int) i;
    }
  }  
  //this doesn't seem to work 
  bool tr = true;
  const cv::Scalar color(255,255,255);
  //fprintf(stderr, "Largest Contour Size : %d\n",(int) max_size);
  if(largest_countour_id >=0){
    vector<cv::Point> c_contour = contours[largest_countour_id];

    std::vector<cv::Point> hull;
    cv::convexHull(c_contour, hull);
    
    for(int j=1; j < c_contour.size(); j++){
      cv::line(mask_count, c_contour[j-1], c_contour[j], color,  2);
    }
    /*for(int j=0; j < c_contour.size(); j++){
      //one way is to directly set the value - but then you need to find the index x + y * rows
      //mask_count.data[]
      mask_count.at<uint8_t>(c_contour[j].y, c_contour[j].x) = mask.at<uint8_t>(c_contour[j].y, c_contour[j].x);
      }*/
    /*for(int j=0; j < hull.size(); j++){
      mask_count.at<bool>(hull[j].y, hull[j].x) = mask.at<bool>(hull[j].y, hull[j].x);
      }*/

    //Ideally we should use the propoer convex hull stuff - but its messing up because of the 
    //car's rollbar 
    
    cv::Mat hull_contour = cv::Mat::zeros(size, CV_8U);
    const int hull_count = (int)hull.size();
    const cv::Point* hull_pts = &hull[0];//&hull[0];
    cv::fillPoly(hull_contour, &hull_pts, &hull_count, 1, cv::Scalar(255));

    cv::imshow("Hull Mask", hull_contour);


    filled_contour = cv::Mat::zeros(size, CV_8U);
    const int filled_count = (int)c_contour.size();
    const cv::Point* filled_pts = &c_contour[0];//&hull[0];
    cv::fillPoly(filled_contour, &filled_pts, &filled_count, 1, cv::Scalar(255));
    
    cv::imshow("Contour Mask", mask_count);
    //cv::imshow("Contour Fill", filled_contour.clone());
  }
  else{
    fprintf(stderr, "Error - No contour found\n");
  }
  //cv::Mat1b sm_display = mask_new.clone();
  
  /*vector<cv::Point> c_countour = contours[i];
    fprintf(stderr, "No of points in contour : %d\n", contours[i].size());
    for(int j=0; j < c_countour.size(); j++){
    fprintf(stderr, "\t (%d,%d)\n", c_countour[j].x, c_countour[j].y);
    }*/
}


void detect_road(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat &hsv_img)
{
  if(img.empty())
    std::cerr << "Error Image empty " << std::endl;
  /*if (hsv_img.empty() || hsv_img.rows !=  img.rows || img.cols != hsv_img.cols)
    hsv_img.create(msg->height, msg->width, CV_8UC3);*/

  cv::Mat hsv; 
  //hsv will be type double
  cvtColor(img, hsv, CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  assert(channels.size() == 3);
  cv::Mat hue = channels[0]; 
  cv::Mat val = channels[1];
  cv::Mat sat = channels[2];

  //cv::Mat h_display = hue.clone();

  //this is how you draw a line - do it on the cloned cv::Mat
  /*cv::Point start(0,0);
  cv::Point end(20,20);
  const cv::Scalar color(0,0,255);
  cv::line(h_display, start, end, color, 3);*/
  //cv::imshow("HUE", h_display);

  //cv::Mat v_display = val.clone();
  //cv::imshow("VAL", v_display);

  //cv::Mat s_display = sat.clone();
  //cv::imshow("SAT", s_display);

  //build a filled contour - and skip those for the road mask 
  
  //0,129 - 0,372 - 127-543 - 228, 543
  std::vector<cv::Point> vehicle_mask_points;
  vehicle_mask_points.push_back(cv::Point(0, 124));
  vehicle_mask_points.push_back(cv::Point(0, 372));
  vehicle_mask_points.push_back(cv::Point(127, 543));
  vehicle_mask_points.push_back(cv::Point(228, 543));

  cv::Size size(img.cols, img.rows);
  cv::Mat vehicle_mask = cv::Mat::zeros(size, CV_8U);
  const int vm_count = (int)vehicle_mask_points.size();
  const cv::Point* vm_pts = &vehicle_mask_points[0];//&hull[0];
  cv::fillPoly(vehicle_mask, &vm_pts, &vm_count, 1, cv::Scalar(255));
  cv::imshow("Vehicle Mask", vehicle_mask);
  

  cv::Mat1b mask = (vehicle_mask == 0 & val <= 255 * 0.05 & sat > 255* 0.1 & sat < 255 * 0.4);// & sa > 0.1 & val < 0.4);  
  cv::Mat filled_contour_basic;
  find_contours(mask, filled_contour_basic);

  /*if(!filled_contour_basic.empty()){
    cv::Mat1b mask_filled = (filled_contour_basic > 0);
    project_to_ground(self, utime, img, mask_filled, self->lcmgl_basic);
  }  
  else{
    //this mask works 
    project_to_ground(self, utime, img, mask, self->lcmgl_basic);
    }*/
  
  cv::Mat1b mask_dil;
  dilate_mask(mask, mask_dil);
  cv::Mat filled_contour;
  find_contours(mask_dil, filled_contour);
  if(!filled_contour.empty()){
    cv::imshow("Contour Fill", filled_contour.clone());
    cv::Mat1b mask_filled = (filled_contour > 0);
    project_to_ground(self, utime, img, mask_filled, self->lcmgl);
  }  
  else{
    //this mask works 
    project_to_ground(self, utime, img, mask, self->lcmgl);
  }

  cv::Mat1b sm_display = mask.clone();
  cv::imshow("Basic Road Detection", sm_display);
  
  //remove the visible car parts from the mask
  

  //cv::Mat thresh; 

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
    cv::cvtColor(img, img, CV_RGB2BGR);
    break;
  case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
    // for some reason msg->row_stride is 0, so we use msg->width instead.
    //does this look ok? - this will loose the color?
    jpeg_decompress_8u_gray(msg->data,
			    msg->size,
			    img.data,
			    msg->width,
			    msg->height,
			    msg->width);
    break;
  case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
    memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height);
    break;
  default:
    fprintf(stderr, "Unrecognized image format\n");
    break;
  }
  return;
}

void setup_projection(Terrain *self, int width, int height){//cv::Mat road){
  BotTrans cam_to_body;
  bot_frames_get_trans(self->frames, self->coord_frame, "body", //bot_frames_get_root_name(self->frames),
		       &cam_to_body);

  int xstep = 1;//4;
  int ystep = 1;//2;
  int ncols = width / xstep + 1;
  int nrows = height / ystep + 1;
  self->img_nvertices = ncols * nrows;
  int img_data_size = self->img_nvertices * sizeof(ImageVertex);
  self->vertices = (ImageVertex*) malloc(img_data_size);
  self->n_vert_indices = (ncols - 1) * (nrows - 1) * 4;
  //cr->n_vert_indices = cr->width / xstep * cr->height / ystep * 4;
  self->vert_indices = (int*) malloc(self->n_vert_indices * sizeof(int));
  int vi_count = 0;
  
  // allocate workspace for projecting the image onto the ground plane
  self->pixel_rays_local = (point3d_t*) malloc(self->img_nvertices * sizeof(point3d_t));
  self->ground_projections_local = (point2d_t*) malloc(self->img_nvertices * sizeof(point2d_t));
  
  ImageVertex *v = self->vertices;
  
  //this projection doesn't change 
  for (int row = 0; row < nrows; row++) {
    int y = row * ystep;
    for (int col = 0; col < ncols; col++) {
      int x = col * xstep;
      double ray_cam[3], ray_body[3];
      bot_camtrans_unproject_pixel(self->camtrans, x, y, ray_cam);

      bot_vector_normalize_3d(ray_cam);
      bot_trans_rotate_vec(&cam_to_body, ray_cam, ray_body);

      v->tx = x;
      v->ty = y;
      v->vx = ray_cam[0];
      v->vy = ray_cam[1];
      v->vz = ray_cam[2];

      v++;

      if (row < nrows - 1 && col < ncols - 1) {
	self->vert_indices[vi_count + 0] = row * ncols + col;
	self->vert_indices[vi_count + 1] = row * ncols + col + 1;
	self->vert_indices[vi_count + 2] = (row + 1) * ncols + col + 1;
	self->vert_indices[vi_count + 3] = (row + 1) * ncols + col;
	
	vi_count += 4;
      }
    }
  }
}

void project_to_ground(Terrain *self, const bot_core_image_t *img){//int utime){
  BotTrans cam_to_local;
  bot_frames_get_trans_with_utime(self->frames, self->coord_frame, bot_frames_get_root_name(self->frames),
				  img->utime, &cam_to_local);

  bot_lcmgl_point_size(self->lcmgl, 3);
  bot_lcmgl_begin(self->lcmgl, GL_POINTS);

  // project image onto the ground plane
  for (int i = 0; i < self->img_nvertices; i+=8) {
    ImageVertex *v = &self->vertices[i];
    //get the ray in local frame 
    self->pixel_rays_local[i].x = v->vx;
    self->pixel_rays_local[i].y = v->vy;
    self->pixel_rays_local[i].z = v->vz;
    double v_cam[3] = { v->vx, v->vy, v->vz };

    int index = v->tx + v->ty* (img->width);
    uint8_t r = img->data[3 *index];
    uint8_t g = img->data[3 *index+1];
    uint8_t b = img->data[3 *index+2];

    bot_trans_rotate_vec(&cam_to_local, v_cam, point3d_as_array(&self->pixel_rays_local[i]));
    
    //check where it hits the ground plane
    if (0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec), &self->pixel_rays_local[i], 0,
					   &self->ground_projections_local[i])) {
      self->ground_projections_local[i].x = NAN;
      self->ground_projections_local[i].y = NAN;
      continue;
    }

    double dist_sq = geom_point_point_distance_squared_2d(&self->ground_projections_local[i],
							  POINT2D(cam_to_local.trans_vec));
    if (dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
      self->ground_projections_local[i].x = NAN;
      self->ground_projections_local[i].y = NAN;
    }

    
    bot_lcmgl_color3f(self->lcmgl, r/255.0, g/255.0, b/255.0);
    bot_lcmgl_vertex3f(self->lcmgl, self->ground_projections_local[i].x, self->ground_projections_local[i].y, 0);
  }
  bot_lcmgl_end(self->lcmgl);
  bot_lcmgl_switch_buffer(self->lcmgl);
  //done projecting 
  //lets draw some stuff 
  
}

void project_to_ground(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat1b& mask, bot_lcmgl_t *lcmgl){//int utime){
  //we just need a new car frame - defined - with which the head is always tracked to 
  BotTrans cam_to_local;
  bot_frames_get_trans_with_utime(self->frames, self->coord_frame, bot_frames_get_root_name(self->frames),
				  utime, &cam_to_local);

  bot_lcmgl_point_size(lcmgl, 3);
  bot_lcmgl_begin(lcmgl, GL_POINTS);

  // project image onto the ground plane
  for (int i = 0; i < self->img_nvertices; i+=8) {
    ImageVertex *v = &self->vertices[i];

    bool road = mask.at<bool>(v->ty, v->tx);
    if(!road)
      continue;
    //get the ray in local frame 
    self->pixel_rays_local[i].x = v->vx;
    self->pixel_rays_local[i].y = v->vy;
    self->pixel_rays_local[i].z = v->vz;
    double v_cam[3] = { v->vx, v->vy, v->vz };

    
    /*int index = v->tx + v->ty* (img->width);
      uint8_t r = img->data[3 *index];
      uint8_t g = img->data[3 *index+1];
      uint8_t b = img->data[3 *index+2];*/
    uint8_t r = 0.0;
    uint8_t g = 1.0;
    uint8_t b = 0.0;

    bot_trans_rotate_vec(&cam_to_local, v_cam, point3d_as_array(&self->pixel_rays_local[i]));
    
    //check where it hits the ground plane
    if (0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec), &self->pixel_rays_local[i], 0,
					   &self->ground_projections_local[i])) {
      self->ground_projections_local[i].x = NAN;
      self->ground_projections_local[i].y = NAN;
      continue;
    }

    double dist_sq = geom_point_point_distance_squared_2d(&self->ground_projections_local[i],
							  POINT2D(cam_to_local.trans_vec));
    if (dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
      self->ground_projections_local[i].x = NAN;
      self->ground_projections_local[i].y = NAN;
    }

    
    bot_lcmgl_color3f(lcmgl, r/255.0, g/255.0, b/255.0);
    bot_lcmgl_vertex3f(lcmgl, self->ground_projections_local[i].x, self->ground_projections_local[i].y, 0);
  }
  bot_lcmgl_end(lcmgl);
  bot_lcmgl_switch_buffer(lcmgl);
  //done projecting 
  //lets draw some stuff 
  
}

void on_image(const lcm_recv_buf_t *rbuf, const char * channel, const bot_core_image_t * msg, void * user) {  
  Terrain *state = (Terrain *) user;
  fprintf(stderr, ".");
  //g_mutex_lock(self->mutex);
  //self->image_list.push_back(bot_core_image_t_copy(msg));
  //state->last_image = bot_core_image_t_copy(msg);
  if(state->img_nvertices== 0){
    fprintf(stderr, "Setting up the projection stuff - only done once\n");
    fprintf(stderr, "Width : %d Height : %d\n", msg->width, msg->height);
    setup_projection(state, msg->width, msg->height);
  }  

  //project_to_ground(state, msg);

  if (state->img.empty() || state->img.rows != msg->height || state->img.cols != msg->width) { 
    if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) { 
      std::cerr << "ERROR: Incoming image is grayscale!! Cannot perform road detection!!!" << std::endl;
      assert(0);
    } else { 
      std::cerr << "One time creation of image" << std::endl;
      state->img.create(msg->height, msg->width, CV_8UC3);
    }
  }
  decode_image(msg, state->img);    

  //this should get the mask and also find the largest road segment 
  detect_road(state, msg->utime, state->img, state->hsv_img);    
  
  //get the bot trans and project the image to the ground 
  //project the image (atleast the road 

  state->img_utime = msg->utime; 
  cv::Mat display = state->img.clone();
  
  cv::imshow("Camera", display);
  return;
}

/*void sift_extractor(bot_core_image_t *bc_img){
  fprintf(stderr, "Extracting Sift\n");
    
  IplImage* img;

  static bot_core_image_t * localFrame=(bot_core_image_t *) calloc(1,sizeof(bot_core_image_t));
  const bot_core_image_t * newFrame=NULL;
  if (bc_img->pixelformat==BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG){
  //create space for decompressed image
  //envoy_decompress_bot_core_image_t(s->received_image,localFrame);
  return;
  //copy pointer for working locally
  newFrame = localFrame;
  }
  else{
  //uncompressed, just copy pointer
  newFrame = bc_img;
  }  
    

  //better copy this - otherwise will crash stuff
  CvMat cvImg = opencv_utils::get_opencv_header_for_bot_core_image_t(newFrame);

  CvMat *frame = &cvImg;//NULL;
  //Copy(&cvImg,frame);

  static IplImage tmpHeader;
  IplImage * currentFrame;

  currentFrame = cvGetImage(frame,&tmpHeader);

  const cv::Mat input = cv::Mat(currentFrame, false);

  //cv::SiftFeatureDetector detector;
  //cv::StarFeatureDetector detector;
  //cv::FastFeatureDetector detector;
  cv::SurfFeatureDetector surf_detector;
  std::vector<cv::KeyPoint> surf_keypoints;
  surf_detector.detect(input, surf_keypoints);

  // Add results to image and save.
  cv::Mat surf_output;
  cv::drawKeypoints(input, surf_keypoints, surf_output);
  cv::imwrite("surf_result.jpg", surf_output);

  cv::StarFeatureDetector star_detector;
  std::vector<cv::KeyPoint> star_keypoints;
  star_detector.detect(input, star_keypoints);

  // Add results to image and save.
  cv::Mat star_output;
  cv::drawKeypoints(input, star_keypoints, star_output);
  cv::imwrite("star_result.jpg", star_output);

  cv::SiftFeatureDetector sift_detector;
  std::vector<cv::KeyPoint> sift_keypoints;
  sift_detector.detect(input, sift_keypoints);

  // Add results to image and save.
  cv::Mat sift_output;
  cv::drawKeypoints(input, sift_keypoints, sift_output);
  cv::imwrite("sift_result.jpg", sift_output);
  }
*/
/*
  bool
  HistogramTracker::initialize(const cv::Mat& img, const cv::Mat& mask) {
  
  hue_info = HistogramInfo();
  val_info = HistogramInfo();
  sat_info = HistogramInfo();
  object_roi = cv::Mat();
  internal_init();

  // Compute mask roi for debug
  if (!computeMaskROI(img, mask, pred_win)) 
  return false;

  // Convert to HSV space
  cv::Mat hsv; 
  cvtColor(img, hsv, CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  assert(channels.size() == 3);
  cv::Mat hue = channels[0]; 
  cv::Mat val = channels[1];
  cv::Mat sat = channels[2];
  // std::cerr << "hue: " << hue << std::endl;

  // Calculate Histogram
  calcHist(&hue, 1, 0, mask, hue_info.histogram, 1, &hue_info.size, &hue_info.pranges); 
  calcHist(&val, 1, 0, mask, val_info.histogram, 1, &val_info.size, &val_info.pranges); 
  calcHist(&sat, 1, 0, mask, sat_info.histogram, 1, &sat_info.size, &sat_info.pranges); 

  // std::cerr << "hue hist: " << hue_info.histogram << " " << std::endl;
  // std::cerr << "val hist: " << val_info.histogram << " " << std::endl;

  normalize(hue_info.histogram, hue_info.histogram, 0, 1, CV_MINMAX);
  normalize(val_info.histogram, val_info.histogram, 0, 1, CV_MINMAX);
  normalize(sat_info.histogram, sat_info.histogram, 0, 1, CV_MINMAX);

  // Compute unimodal histogram
  hue_info.computeUnimodalHistogram();
  val_info.computeUnimodalHistogram();
  sat_info.computeUnimodalHistogram();

  // Create debug histograms
  hue_info.createHistogramImage();
  val_info.createHistogramImage();
  sat_info.createHistogramImage();

  // cv::Mat display = img.clone();
  // showHistogramInfo(display);

  // cv::imshow("Initialize Histogram Tracker", display);
  mask_initialized_ = true;

  // Once mask is inialized, find the 
  std::cerr << "Initialized: Prediction window " << pred_win.tl() << "->" << pred_win.br() << std::endl;

  return true;
  }


*/


int detect_road(Terrain *self, bot_core_image_t *img){
  //doing nothing for now 
  if(img->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG){
    fprintf(stderr,"JPEG - Skipping for now\n");
    return -1;
  }

  //better copy this - otherwise will crash stuff
  int64_t stime = bot_timestamp_now();
  CvMat cvImg = opencv_utils::get_opencv_header_for_bot_core_image_t(img);
  
  static IplImage tmpHeader;
  IplImage * currentFrame;
  CvMat *frame = &cvImg;
  currentFrame = cvGetImage(frame,&tmpHeader);
  
  const cv::Mat input = cv::Mat(currentFrame, false);

  //  Mat mtx(img);
  //void cvtColor(const Mat& src, Mat& dst, int code, int dstCn=0)
  cv::Mat bgr_mat(img->height,img->width,CV_8UC3);
  cvtColor(input, bgr_mat, CV_RGB2BGR);

  cv::Mat hsv_mat(img->height,img->width,CV_8UC3);
  cvtColor(input, hsv_mat, CV_RGB2HSV);
  int64_t etime = bot_timestamp_now();
  fprintf(stderr, "Time taken : %f\n", (etime-stime)/1.0e6);
  opencv_utils::DisplayFrame disp("Test", &cvImg, img->width);
  disp.display();
  //self->dp.display("RGB", &cvImg, img->width);

  return 0;
}

static void *p_thread(void *user)
{
  Terrain * self = (Terrain *) user;

  int status = 0;
  while(1){
    g_mutex_lock(self->mutex);
    if(!self->last_image){
      usleep(100);
      g_mutex_unlock(self->mutex);
    }
    else{
      bot_core_image_t *img = self->last_image;
      self->last_image = NULL;
      g_mutex_unlock(self->mutex);
      //int status = detect_road(self, img);
      //process the last one??
      //fprintf(stderr, "Buffer size : %d\n", self->image_list.size());
    }

    /*if(self->image_list.size() == 0){
      usleep(5000);
      }
      else{
      //process the last one??
      //fprintf(stderr, "Buffer size : %d\n", self->image_list.size());
      }*/
    /*if(s->processed){
     
      }
      else{
      int status = detect_objects(s);
      g_mutex_lock(s->mutex);
      s->processed = 1;
      g_mutex_unlock(s->mutex);
      }*/
  }
}

int main(int argc, char **argv)
{
  g_thread_init(NULL);
  setlinebuf (stdout);

  Terrain *state = new Terrain();

  state->lcm =  bot_lcm_get_global(NULL);
  state->last_image = NULL;
  state->param = bot_param_new_from_server(state->lcm, 1);
  state->frames = bot_frames_get_global (state->lcm, state->param);

  state->mutex = g_mutex_new();

  state->mainloop = g_main_loop_new( NULL, FALSE );  
    
  fprintf(stderr, "Setting the segment first method - this one works best\n");

  ConciseArgs opt(argc, (char**)argv);
  opt.add(state->options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
  opt.add(state->options.vDEBUG, "d", "debug","Debug mode");
  opt.parse();

  state->lcmgl_basic = bot_lcmgl_init(state->lcm, "Terrain-detection-Basic");
  state->lcmgl = bot_lcmgl_init(state->lcm, "Terrain-detection");

  /*const char *optstring = "vhcfsdb";
    struct option long_opts[] = { { "help", no_argument, 0, 'h' },
    { "verbose", no_argument, 0, 'v' }, 
    { 0, 0, 0, 0 } };

    int c;
    while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
    case 'h':
    //usage(argv[0]);
    fprintf(stderr, "No help found - please add help\n");
    break;
    case 'v':
    {
    fprintf(stderr,"Verbose\n");
    state->verbose = 1;
    break;
    }
    }
    }*/

  char * cam_name = bot_param_get_camera_name_from_lcm_channel(state->param,  state->options.vCHANNEL.c_str());
  if (cam_name != NULL) {
    state->camtrans = bot_param_get_new_camtrans(state->param, cam_name);
    state->coord_frame = bot_param_get_camera_coord_frame(state->param, cam_name);
      
    /*if (state->camtrans) {
      double xscale = st->width / bot_camtrans_get_image_width(cr->camtrans);
      double yscale = cr->width / bot_camtrans_get_image_width(cr->camtrans);
      assert(fabs(xscale - yscale) < 1e-6);
      bot_camtrans_scale_image(cr->camtrans, xscale);
      }
      else {
      printf("%s:%d couldn't find calibration parameters for %s\n", __FILE__, __LINE__, cam_name);
      }*/
    free(cam_name);
  }

  bot_core_image_t_subscribe(state->lcm, state->options.vCHANNEL.c_str(), on_image, state);

  //cvNamedWindow( "Draw Maks");
  // Main lcm handle
  while(1) { 
    unsigned char c = cv::waitKey(1) & 0xff;
    lcm_handle(state->lcm);

    /*if(state->last_image){
      int64_t stime = bot_timestamp_now();
      CvMat cvImg = opencv_utils::get_opencv_header_for_bot_core_image_t(state->last_image);
  
      static IplImage tmpHeader;
      IplImage * currentFrame;
      CvMat *frame = &cvImg;
      currentFrame = cvGetImage(frame,&tmpHeader);

      cvShowImage( "Draw Mask",  currentFrame);
      }*/
    /*if(!state->img.empty()){
      //vector
      fprintf(stderr, "Writing Image\n");
      imwrite("camera_image.jpg", state->img);
     }*/

    if (c == 'q') {
      break;  
    } else if ( c == 'c' ) {      
      if (state->options.vDEBUG) { 
	//cv::Mat1b maskd = mask.clone();
	cv::imshow("Captured Image", state->img);
	//cv::imshow("Captured Mask", maskd);
      }
      // Initialize with image and mask
      //state->tracker->initialize(state->img, mask);
    }
  }

  return 0;
  
  /*if (!state->mainloop){
    printf("Couldn't create main loop\n");
    return -1;
    }

    //add lcm to mainloop 
    bot_glib_mainloop_attach_lcm (state->lcm);

 
  
    pthread_create(&state->process_thread , NULL, p_thread, state);

    state->dp.start();
    
    //adding proper exiting 
    bot_signal_pipe_glib_quit_on_kill (state->mainloop);
    
    fprintf(stderr, "Starting Main Loop\n");

    ///////////////////////////////////////////////
    g_main_loop_run(state->mainloop);
    
    bot_glib_mainloop_detach_lcm(state->lcm);*/
}

