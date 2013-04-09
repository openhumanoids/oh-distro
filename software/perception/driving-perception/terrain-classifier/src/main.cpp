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
#include <occ_map/PixelMap.hpp>

#define MAX_GROUND_PROJECTION_DISTANCE 30//120
#define MAX_GROUND_PROJECTION_DISTANCE_SQ (MAX_GROUND_PROJECTION_DISTANCE*MAX_GROUND_PROJECTION_DISTANCE)

using namespace std;
using namespace occ_map;

struct RoadDetectorOptions { 
  bool vDEBUG;
  std::string vCHANNEL;
  RoadDetectorOptions () : 
    vCHANNEL(std::string("CAMERALEFT")), vDEBUG(false) {}
};

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

  //these are not used right now 
  vector<bot_core_image_t *> image_list;
  bot_core_image_t *last_image;
  
  std::vector<cv::Point> vehicle_mask_points;

  cv::Mat img, hsv_img;
  cv::Mat vehicle_mask;
  int64_t img_utime;
};

void create_pixelmap(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat1b& mask, PixelMap<float>* distance_map, bot_lcmgl_t *lcmgl=NULL);

void create_contour_pixelmap(Terrain *self, int64_t utime, cv::Mat& mask, PixelMap<float>* distance_map, vector<cv::Point>contour, bot_lcmgl_t *lcmgl = NULL);

void project_to_ground(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat1b& mask, bot_lcmgl_t *lcmgl);

void dilate_mask(cv::Mat1b& mask, cv::Mat1b& mask_new){

  //cvFindContours
  //offset 3 is good to remove just basic noise 
  
  //OFFSET 5 is good to remove the centerline 
  const int OFFSET = 5;
  cv::dilate(mask, mask_new, cv::Mat(), cv::Point(-1,-1), OFFSET);

  if(0){
    cv::Mat1b sm_display = mask_new.clone();
    cv::imshow("Dilated Mask", sm_display);
  }
}

void find_contours(cv::Mat1b& mask, cv::Mat& filled_contour, vector<cv::Point>&largest_contour){

  cv::Size size(mask.cols, mask.rows);
  vector<vector<cv::Point> > contours;

  // find contours and store them all as a list
  cv::Mat1b mask_new = mask.clone();
  cv::findContours(mask_new, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  
  //fprintf(stderr, "No of countours : %d\n", contours.size());

  vector<cv::Point> approx;

  cv::Mat1b mask_count = cv::Mat1b::zeros(mask_new.rows, mask_new.cols);

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

  bool tr = true;
  const cv::Scalar color(255,255,255);
  //fprintf(stderr, "Largest Contour Size : %d\n",(int) max_size);
  if(largest_countour_id >=0){
    largest_contour = contours[largest_countour_id];

    //we should use this?? 

    std::vector<cv::Point> hull;
    cv::convexHull(largest_contour, hull);
    
    for(int j=1; j < largest_contour.size(); j++){
      cv::line(mask_count, largest_contour[j-1], largest_contour[j], color,  2);
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
    const int filled_count = (int)largest_contour.size();
    const cv::Point* filled_pts = &largest_contour[0];//&hull[0];
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

// Broken - use distanceTransform instead
void get_distance_mask(cv::Mat& edges, cv::Mat& mask){
  //0's anywhere the mask had 1 - which is where we need to fill 
  cv::Size size(mask.cols, mask.rows);
  cv::Mat dist_from_edge = cv::Mat::zeros(size, CV_32FC3);

  cv::Mat1b mask_count = (mask==0);//cv::Mat1b::zeros(mask.rows, mask.cols);
  vector<cv::Point> edge_points;
  //lets get the points for which edges are 1
  for(int i=0; i < edges.cols; i++){
    for(int j=0; j < edges.rows; j++){
      if(edges.at<uint8_t>(i,j)>0){
	edge_points.push_back(cv::Point(i,j));
	//edge point
      }
    }
  }

  //do we just go from the points - vs distance 
  for(size_t k=0; k< edge_points.size(); k++){
    for(uint8_t i=0; i< 20; i++){
      for(uint8_t j=0; j< 20; j++){
	cv::Point pt = edge_points[k];
	//try all 4 points 
	int dx = pt.x + i;
	int dy = pt.y + j;
	if(dx < mask.cols && dy < mask.rows){
	  //check if filled 
	  if(mask_count.at<uint8_t>(dx,dy)==0){
	    //put the value
	    dist_from_edge.at<float>(dx,dy) = hypot(dx,dy);
	    mask_count.at<uint8_t>(dx,dy) = (uint8_t) 1;
	  }
	}
	dx = pt.x + i;
	dy = pt.y - j;
	if(dx < mask.cols && dy >=0){
	  //check if filled 
	  if(mask_count.at<uint8_t>(dx,dy)==0){
	    //put the value
	    dist_from_edge.at<float>(dx,dy) = hypot(dx,dy);
	    mask_count.at<uint8_t>(dx,dy) = (uint8_t) 1;
	  }
	}
	dx = pt.x - i;
	dy = pt.y + j;
	if(dx >=0 && dy < mask.rows){
	  //check if filled 
	  if(mask_count.at<uint8_t>(dx,dy)==0){
	    //put the value
	    dist_from_edge.at<float>(dx,dy) = hypot(dx,dy);
	    mask_count.at<uint8_t>(dx,dy) = (uint8_t) 1;
	  }
	}
	dx = pt.x - i;
	dy = pt.y - j;
	if(dx >=0 && dy >=0){
	  //check if filled 
	  if(mask_count.at<uint8_t>(dx,dy)==0){
	    //put the value
	    dist_from_edge.at<float>(dx,dy) = hypot(dx,dy);
	    mask_count.at<uint8_t>(dx,dy) = (uint8_t) 1;
	  }
	}
      }
    }
  }

  cv::imshow("Distance Fill", dist_from_edge.clone());
}


//Routine that does the road detection//
//(1) convert the image to HSV space
//(2) Apply (for now hard coded) HSV Filter
//    (i)  Need to add one that classifies the road divider color as valid road also 
//    (ii) Applies a mask that removes the car in the image (right now hard coded)
//(3) Dilate the result 
//(4) Find the largest contour
//(5) Project the road mask to the estimated ground plane - the road should be on the ground level 
//(6) Apply distance transform (such that points away from the road edge will have lower cost
//(7) Also detect obstacles (such as traffic cones) on the road - by doing a diff 
//(8) Create and publish the cost map - for use by the controller - controller can handle getting the minimal cost route through the cost map 

int detect_road(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat &hsv_img)
{
  if(img.empty()){
    std::cerr << "Error: Image empty " << std::endl;
    return -1;
  }
  if(self->vehicle_mask.empty()){
    std::cerr << "Error: Vehicle mask empty " << std::endl;
    return -2;
  }

  cv::Mat hsv; 
  //hsv will be type double
  cvtColor(img, hsv, CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  assert(channels.size() == 3);
  cv::Mat hue = channels[0]; 
  cv::Mat val = channels[1];
  cv::Mat sat = channels[2];

  if(state->options.vDEBUG){
    //display the HSV results 
    cv::Mat h_display = hue.clone();
    cv::imshow("HUE", h_display);

    cv::Mat v_display = val.clone();
    cv::imshow("VAL", v_display);

    cv::Mat s_display = sat.clone();
    cv::imshow("SAT", s_display);
  }

  //we might need another filtering part for the yellow road lines
  cv::Mat1b mask = (self->vehicle_mask == 0 & val <= 255 * 0.05 & sat > 255* 0.1 & sat < 255 * 0.4);

  //the dilation is need for the contour to work - otherwise it will pick only a part of the road sometimes 
  cv::Mat1b mask_dil;
  dilate_mask(mask, mask_dil);

  //get the contour points and the contour 
  cv::Mat filled_contour;
  vector<cv::Point> largest_contour;
  find_contours(mask_dil, filled_contour, largest_contour);

  if(!filled_contour.empty()){
    cv::imshow("Contour Fill", filled_contour.clone());
    cv::Mat1b mask_filled = (filled_contour > 0);

    cv::Mat contours;
    cv::Canny(mask_filled, contours,10,350);
    
    //we can use the canny edges to get the distance from edges ?? - to build up the cost map 
    cv::imshow("Canny Edges", contours.clone());

    //get_distance_mask(contours, mask_filled);
    //figure this out 
    //we should use this if we can - but projected on to the ground plane 
    //also in the groud plane - we should prob draw lines between points 

    cv::Mat dist;
    distanceTransform(mask_filled, dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);//CV_DIST_MASK_5);
    
    dist *= 5000;
    pow(dist, 0.5, dist);
    
    cv::Mat dist32s, dist8u, dist8u1, dist8u2;
    
    dist.convertTo(dist32s, CV_32S, 1, 0.5);
    dist32s &= cv::Scalar::all(255);
    
    dist32s.convertTo(dist8u1, CV_8U, 1, 0);
    dist32s *= -1;
    
    dist32s += cv::Scalar::all(255);
    dist32s.convertTo(dist8u2, CV_8U);
    
    cv::Mat planes[] = {dist8u1, dist8u2, dist8u2};
    merge(planes, 3, dist8u);
    
    cv::imshow("Distance Transform", dist8u);
        
    PixelMap<float>* contour_map;
    create_contour_pixelmap(self, utime, mask_filled, contour_map, largest_contour, self->lcmgl);

    //project_to_ground(
    /*PixelMap<float>* distance_map;
      create_pixelmap(self, utime, img, mask_filled, distance_map, self->lcmgl);*/
    //
    //occ_map_pixel_map_t_publish(self->lcm, "TERRAIN_COST", map_msg);    
    //delete distance_map;
  }  
  else{
    //this mask works 
    project_to_ground(self, utime, img, mask, self->lcmgl);
  }

  cv::Mat1b sm_display = mask.clone();
  cv::imshow("Basic Road Detection", sm_display);
  
  //remove the visible car parts from the mask
  

  //cv::Mat thresh; 

  return 0;
}

void
decode_image(const bot_core_image_t * msg, cv::Mat& img)
{
  if (img.empty() || img.rows != msg->height || img.cols != msg->width)
    img.create(msg->height, msg->width, CV_8UC3);

  // extract image data
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

void create_contour_pixelmap(Terrain *self, int64_t utime, cv::Mat& mask, PixelMap<float>* distance_map, vector<cv::Point>contour, bot_lcmgl_t *lcmgl){

  //this should all be done in a body relative frame - which will break the rendering of the occ map - but what the hell :D
  
  BotTrans head_to_local;

  bot_frames_get_trans_with_utime(self->frames, "head", bot_frames_get_root_name(self->frames),
				  utime, &head_to_local);
  
  BotTrans cam_to_local;

  bot_frames_get_trans_with_utime(self->frames, self->coord_frame, bot_frames_get_root_name(self->frames),
				  utime, &cam_to_local);

  //fprintf(stderr, "Done with Pixel Map - No of Cells : %d\n", distance_map->num_cells);
  //We need a transform from the Cam to center of Vehicle - which is where we are trying to drive from 
  if(lcmgl){
    bot_lcmgl_point_size(lcmgl, 10);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
  }

  BotTrans corner_to_head;
  
  double rpy[3] = {0};
  corner_to_head.trans_vec[0] = 0;
  corner_to_head.trans_vec[1] = -10;
  corner_to_head.trans_vec[2] = 0;

  bot_roll_pitch_yaw_to_quat(rpy, corner_to_head.rot_quat);

  double min_xy[2] = {10000, 10000};
  double max_xy[2] = {-10000, -10000};
  
  BotTrans corner_to_local;
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);
  
  /*if(lcmgl){
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
    bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);
    }*/
  //double xy0[2] = {corner_to_local.trans_vec[0], corner_to_local.trans_vec[1]};

  corner_to_head.trans_vec[0] = 25;
  corner_to_head.trans_vec[1] = -10;
  
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

  /*if(lcmgl){
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
    bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);
    }*/

  corner_to_head.trans_vec[0] = 25;
  corner_to_head.trans_vec[1] = 10;
  
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

  /*if(lcmgl){
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
    bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);
    }*/

  corner_to_head.trans_vec[0] = 0;
  corner_to_head.trans_vec[1] = 10;
  
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

  /*if(lcmgl){
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
    bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);
    }*/

  if(lcmgl){
    bot_lcmgl_end(lcmgl);
    bot_lcmgl_point_size(lcmgl, 3);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
  }
  

  //double xy1[2] = {corner_to_local.trans_vec[0], corner_to_local.trans_vec[1]};
  
  /*fprintf(stderr, "Min {%.3f, %.3f} - Max {%.3f, %.3f}\n", 
    min_xy[0], min_xy[1], max_xy[0], max_xy[1]);
  */

  //fprintf(stderr, "Creating Pixel Map\n");
  distance_map = new PixelMap<float>(min_xy, max_xy, 0.5, false);//xy0, xy1, 0.5, false);
  distance_map->data = new float[distance_map->num_cells];
  memset(distance_map->data, 0, sizeof(float) * distance_map->num_cells);
  

  BotTrans local_to_cam = cam_to_local;
  bot_trans_invert(&local_to_cam);
  
  BotTrans point_to_local;
  point_to_local.trans_vec[2] = 0;
  bot_roll_pitch_yaw_to_quat(rpy, point_to_local.rot_quat);
  
  BotTrans point_to_cam; 
  int skip = 1;
  // project image onto the ground plane
  int map_count = 0;
  
  //we should do this - but to the projected 
  cv::Mat1b road_outline = (mask >0);//cv::Mat1b::zeros(mask.rows, mask.cols);
  const cv::Scalar color(255,255,255);  
  for(int j=1; j < contour.size(); j++){
    cv::line(road_outline, contour[j-1], contour[j], color,  2);
  }

  for(int i= 0; i < road_outline.cols; i++){
    for(int j= road_outline.rows - 20; j < road_outline.rows; j++){
      road_outline.at<uint8_t>(j,i) = 1;
    }
  }
  
  //give up the bottom set of pixels - because we see the car sometimes 
  
  cv::Size size(mask.cols, mask.rows);
  cv::Mat dist_color = cv::Mat::zeros(size, CV_8U);

  cv::Mat dist;
  distanceTransform(road_outline, dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);
  
  uint8_t max = 0;
  for(int i=0; i < dist.cols; i++){
    for(int j=0; j < dist.rows; j++){
      uint8_t d = 254* fmin(1, 100/dist.at<float>(j,i));
      dist_color.at<uint8_t>(j,i) = d;//uint8_t(1);            
      if(d > max)
	max = d;
    }
  }

  //we would need to local minimum 
  
  fprintf(stderr, "Max : %d\n", max);

  
  
  cv::imshow("Road Distance Transform", dist_color);
  //apply the distance transform here

  cv::imshow("Road Outline", road_outline);

  if(0){
    for(size_t i=1; i < contour.size(); i++){
      cv::Point pt0 = contour[i-1];
      cv::Point pt1 = contour[i];

      int ind1 = pt0.x + pt0.y * mask.cols;
      int ind2 = pt1.x + pt1.y * mask.cols;

      ImageVertex *v1 = &self->vertices[ind1];
      ImageVertex *v2 = &self->vertices[ind2];

      point3d_t pixel_rays_local;
      pixel_rays_local.x = v1->vx;
      pixel_rays_local.y = v1->vy;
      pixel_rays_local.z = v1->vz;

      point2d_t ground_projections_local;

      double v_cam[3] = { v1->vx, v1->vy, v1->vz };
      //project these points - and draw a line between them 

      uint8_t r = 0.0;
      uint8_t g = 1.0;
      uint8_t b = 0.0;

      bot_trans_rotate_vec(&cam_to_local, v_cam, point3d_as_array(&pixel_rays_local));
    
      //check where it hits the ground plane
      if (0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec), &pixel_rays_local, 0,
					     &ground_projections_local)) {
	continue;
      }

      double dist_sq = geom_point_point_distance_squared_2d(&ground_projections_local,
							    POINT2D(cam_to_local.trans_vec));

      if (dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
	continue;
      }

      //now check the current one 
      point3d_t pixel_rays_local2;
      pixel_rays_local2.x = v2->vx;
      pixel_rays_local2.y = v2->vy;
      pixel_rays_local2.z = v2->vz;

      point2d_t ground_projections_local2;

      double v_cam2[3] = { v2->vx, v2->vy, v2->vz };
      //project these points - and draw a line between them 

      bot_trans_rotate_vec(&cam_to_local, v_cam2, point3d_as_array(&pixel_rays_local2));
    
      //check where it hits the ground plane
      if (0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec), &pixel_rays_local2, 0,
					     &ground_projections_local2)) {
	//current ind messed up - then move to next
	i++;
	continue;
      }

      dist_sq = geom_point_point_distance_squared_2d(&ground_projections_local2,
						     POINT2D(cam_to_local.trans_vec));

      if (dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
	i++;
	continue;
      }

      point_to_local.trans_vec[0] = ground_projections_local.x;
      point_to_local.trans_vec[1] = ground_projections_local.y;
    
      double xy_1[2] = {ground_projections_local.x, ground_projections_local.y};
      double xy_2[2] = {ground_projections_local2.x, ground_projections_local2.y};
    
      float clamp[2] = {0,1};

      if(distance_map->isInMap(xy_2)){// && distance_map->isInMap(xy_2)){
	distance_map->writeValue(xy_2, 1.0);
	//distance_map->rayTrace(xy_1, xy_2, 1,1, clamp);
	map_count++;
      }
    
      if(lcmgl){
	bot_lcmgl_color3f(lcmgl, 1.0, 0, 0);
	bot_lcmgl_vertex3f(lcmgl, ground_projections_local.x, ground_projections_local.y, 0);
      }
    }    
  }

  if(0){
    int back_ind = 0;
    
    
    for(size_t i=1; i < contour.size(); i++){
      cv::Point pt0 = contour[back_ind];
      cv::Point pt1 = contour[i];

      int ind1 = pt0.x + pt0.y * mask.cols;
      int ind2 = pt1.x + pt1.y * mask.cols;

      ImageVertex *v1 = &self->vertices[ind1];
      ImageVertex *v2 = &self->vertices[ind2];

      point3d_t pixel_rays_local;
      pixel_rays_local.x = v1->vx;
      pixel_rays_local.y = v1->vy;
      pixel_rays_local.z = v1->vz;

      point2d_t ground_projections_local;

      double v_cam[3] = { v1->vx, v1->vy, v1->vz };
      //project these points - and draw a line between them 

      uint8_t r = 0.0;
      uint8_t g = 1.0;
      uint8_t b = 0.0;

      bot_trans_rotate_vec(&cam_to_local, v_cam, point3d_as_array(&pixel_rays_local));
    
      //check where it hits the ground plane
      if (0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec), &pixel_rays_local, 0,
					     &ground_projections_local)) {
	//if back ind messed up we should move forward 
	back_ind+=1;
	i++;
	continue;
      }

      double dist_sq = geom_point_point_distance_squared_2d(&ground_projections_local,
							    POINT2D(cam_to_local.trans_vec));

      if (dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
	back_ind+=1;
	i++;
	continue;
      }

      //now check the current one 
      point3d_t pixel_rays_local2;
      pixel_rays_local2.x = v2->vx;
      pixel_rays_local2.y = v2->vy;
      pixel_rays_local2.z = v2->vz;

      point2d_t ground_projections_local2;

      double v_cam2[3] = { v2->vx, v2->vy, v2->vz };
      //project these points - and draw a line between them 

      bot_trans_rotate_vec(&cam_to_local, v_cam2, point3d_as_array(&pixel_rays_local2));
    
      //check where it hits the ground plane
      if (0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec), &pixel_rays_local2, 0,
					     &ground_projections_local2)) {
	//current ind messed up - then move to next
	i++;
	continue;
      }

      dist_sq = geom_point_point_distance_squared_2d(&ground_projections_local2,
						     POINT2D(cam_to_local.trans_vec));

      if (dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
	i++;
	continue;
      }

      back_ind = i;

      point_to_local.trans_vec[0] = ground_projections_local.x;
      point_to_local.trans_vec[1] = ground_projections_local.y;
    
      double xy_1[2] = {ground_projections_local.x, ground_projections_local.y};
      double xy_2[2] = {ground_projections_local2.x, ground_projections_local2.y};
    
      float clamp[2] = {0,1};

      if(distance_map->isInMap(xy_1) && distance_map->isInMap(xy_2)){
	distance_map->writeValue(xy_2, 1.0);
	//distance_map->rayTrace(xy_1, xy_2, 1,1, clamp);
	map_count++;
      }
    
      /*if(lcmgl){
	bot_lcmgl_color3f(lcmgl, 1.0, 0, 0);
	bot_lcmgl_vertex3f(lcmgl, ground_projections_local.x, ground_projections_local.y, 0);
	}*/
      //self->ground_projections_local[i].x, self->ground_projections_local[i].y
      /*if(i %8==0){
	if(lcmgl){
	bot_lcmgl_color3f(lcmgl, color[0], color[1], color[2]);//r/255.0, g/255.0, b/255.0);
	bot_lcmgl_vertex3f(lcmgl, self->ground_projections_local[i].x, self->ground_projections_local[i].y, 0);
	}
	}*/

    }
  }

  for (int i = 0; i < self->img_nvertices; i+=skip) {
    ImageVertex *v = &self->vertices[i];

    bool road = mask.at<uint8_t>(v->ty, v->tx);
    
    if(!road)
      continue;
    //get the ray in local frame 
    self->pixel_rays_local[i].x = v->vx;
    self->pixel_rays_local[i].y = v->vy;
    self->pixel_rays_local[i].z = v->vz;
    double v_cam[3] = { v->vx, v->vy, v->vz };

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
      continue;
    }


    point_to_local.trans_vec[0] = self->ground_projections_local[i].x;
    point_to_local.trans_vec[1] = self->ground_projections_local[i].y;
    
    bot_trans_apply_trans_to(&local_to_cam, &point_to_local, &point_to_cam);
    
    //fprintf(stderr, "Dist : %f, %f - %f\n", point_to_cam.trans_vec[0], point_to_cam.trans_vec[1], hypot(point_to_cam.trans_vec[0], point_to_cam.trans_vec[1]));
    
    double dist_rat = fmin(1,dist.at<float>(v->ty, v->tx)/50);//
    //double dist_rat = fmin(1,hypot(point_to_cam.trans_vec[0], point_to_cam.trans_vec[1])/ 5.0);
    float *color = bot_color_util_jet(dist_rat);
    
    double xy_c[2] = {self->ground_projections_local[i].x, self->ground_projections_local[i].y};
    if(distance_map->isInMap(xy_c)){
      distance_map->writeValue(xy_c, dist_rat);
      //fprintf(stderr, "Value at %f,%f ",xy_c[0], xy_c[1]);
      //fprintf(stderr, ":  %f\n", distance_map->readValue(xy_c));//dist_rat);
      map_count++;
    }
    //self->ground_projections_local[i].x, self->ground_projections_local[i].y
    /*if(i %8==0){
      if(lcmgl){
	bot_lcmgl_color3f(lcmgl, color[0], color[1], color[2]);//r/255.0, g/255.0, b/255.0);
	bot_lcmgl_vertex3f(lcmgl, self->ground_projections_local[i].x, self->ground_projections_local[i].y, 0);
      }
      }*/
  }

  const occ_map_pixel_map_t *map_msg = distance_map->get_pixel_map_t(utime);
  occ_map_pixel_map_t_publish(self->lcm, "PIXEL_MAP", map_msg);    

  fprintf(stderr, "Map Count : %d\n", map_count);
  if(lcmgl){
    bot_lcmgl_end(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);
  }  
}

void create_pixelmap(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat1b& mask, PixelMap<float>* distance_map, bot_lcmgl_t *lcmgl){

  //this should all be done in a body relative frame - which will break the rendering of the occ map - but what the hell :D
  
  BotTrans head_to_local;

  bot_frames_get_trans_with_utime(self->frames, "head", bot_frames_get_root_name(self->frames),
				  utime, &head_to_local);
  
  BotTrans cam_to_local;

  bot_frames_get_trans_with_utime(self->frames, self->coord_frame, bot_frames_get_root_name(self->frames),
				  utime, &cam_to_local);

  //fprintf(stderr, "Done with Pixel Map - No of Cells : %d\n", distance_map->num_cells);
  //We need a transform from the Cam to center of Vehicle - which is where we are trying to drive from 
  if(lcmgl){
    bot_lcmgl_point_size(lcmgl, 10);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
  }

  BotTrans corner_to_head;
  
  double rpy[3] = {0};
  corner_to_head.trans_vec[0] = 0;
  corner_to_head.trans_vec[1] = -10;
  corner_to_head.trans_vec[2] = 0;

  bot_roll_pitch_yaw_to_quat(rpy, corner_to_head.rot_quat);

  double min_xy[2] = {10000, 10000};
  double max_xy[2] = {-10000, -10000};
  
  BotTrans corner_to_local;
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);
  
  bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
  bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);

  //double xy0[2] = {corner_to_local.trans_vec[0], corner_to_local.trans_vec[1]};

  corner_to_head.trans_vec[0] = 25;
  corner_to_head.trans_vec[1] = -10;
  
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

  bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
  bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);

  corner_to_head.trans_vec[0] = 25;
  corner_to_head.trans_vec[1] = 10;
  
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

  bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
  bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);

  corner_to_head.trans_vec[0] = 0;
  corner_to_head.trans_vec[1] = 10;
  
  bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

  min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
  min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

  max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
  max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

  bot_lcmgl_color3f(lcmgl, 1, 0, 0);//r/255.0, g/255.0, b/255.0);
  bot_lcmgl_vertex3f(lcmgl, corner_to_local.trans_vec[0], corner_to_local.trans_vec[1], 0);

  if(lcmgl){
    bot_lcmgl_end(lcmgl);
    bot_lcmgl_point_size(lcmgl, 3);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
  }
  

  //double xy1[2] = {corner_to_local.trans_vec[0], corner_to_local.trans_vec[1]};
  
  /*fprintf(stderr, "Min {%.3f, %.3f} - Max {%.3f, %.3f}\n", 
    min_xy[0], min_xy[1], max_xy[0], max_xy[1]);
  */

  //fprintf(stderr, "Creating Pixel Map\n");
  distance_map = new PixelMap<float>(min_xy, max_xy, 0.5, false);//xy0, xy1, 0.5, false);
  distance_map->data = new float[distance_map->num_cells];
  memset(distance_map->data, 0, sizeof(float) * distance_map->num_cells);
  

  BotTrans local_to_cam = cam_to_local;
  bot_trans_invert(&local_to_cam);
  
  BotTrans point_to_local;
  point_to_local.trans_vec[2] = 0;
  bot_roll_pitch_yaw_to_quat(rpy, point_to_local.rot_quat);
  
  BotTrans point_to_cam; 
  int skip = 1;
  // project image onto the ground plane
  int map_count = 0;

  for (int i = 0; i < self->img_nvertices; i+=skip) {
    ImageVertex *v = &self->vertices[i];

    bool road = mask.at<uint8_t>(v->ty, v->tx);
    
    if(!road)
      continue;
    //get the ray in local frame 
    self->pixel_rays_local[i].x = v->vx;
    self->pixel_rays_local[i].y = v->vy;
    self->pixel_rays_local[i].z = v->vz;
    double v_cam[3] = { v->vx, v->vy, v->vz };

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
      continue;
    }


    point_to_local.trans_vec[0] = self->ground_projections_local[i].x;
    point_to_local.trans_vec[1] = self->ground_projections_local[i].y;
    
    bot_trans_apply_trans_to(&local_to_cam, &point_to_local, &point_to_cam);
    
    //fprintf(stderr, "Dist : %f, %f - %f\n", point_to_cam.trans_vec[0], point_to_cam.trans_vec[1], hypot(point_to_cam.trans_vec[0], point_to_cam.trans_vec[1]));
    
    double dist_rat = fmin(1,hypot(point_to_cam.trans_vec[0], point_to_cam.trans_vec[1])/ 5.0);
    float *color = bot_color_util_jet(dist_rat);
    
    double xy_c[2] = {self->ground_projections_local[i].x, self->ground_projections_local[i].y};
    if(distance_map->isInMap(xy_c)){
      distance_map->writeValue(xy_c, dist_rat);
      //fprintf(stderr, "Value at %f,%f ",xy_c[0], xy_c[1]);
      //fprintf(stderr, ":  %f\n", distance_map->readValue(xy_c));//dist_rat);
      map_count++;
    }
    //self->ground_projections_local[i].x, self->ground_projections_local[i].y
    if(i %8==0){
      if(lcmgl){
	bot_lcmgl_color3f(lcmgl, color[0], color[1], color[2]);//r/255.0, g/255.0, b/255.0);
	bot_lcmgl_vertex3f(lcmgl, self->ground_projections_local[i].x, self->ground_projections_local[i].y, 0);
      }
    }
  }

  const occ_map_pixel_map_t *map_msg = distance_map->get_pixel_map_t(utime);
  occ_map_pixel_map_t_publish(self->lcm, "PIXEL_MAP", map_msg);    

  fprintf(stderr, "Map Count : %d\n", map_count);
  if(lcmgl){
    bot_lcmgl_end(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);
  }  
}

void project_to_ground(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat1b& mask, bot_lcmgl_t *lcmgl){
  //we just need a new car frame - defined - with which the head is always tracked to 
  BotTrans cam_to_local;
  bot_frames_get_trans_with_utime(self->frames, self->coord_frame, bot_frames_get_root_name(self->frames),
				  utime, &cam_to_local);

  //We need a transform from the Cam to center of Vehicle - which is where we are trying to drive from 

  bot_lcmgl_point_size(lcmgl, 3);
  bot_lcmgl_begin(lcmgl, GL_POINTS);

  BotTrans local_to_cam = cam_to_local;
  bot_trans_invert(&local_to_cam);
  
  BotTrans point_to_local;
  point_to_local.trans_vec[2] = 0;
  double rpy[3] = {0};
  bot_roll_pitch_yaw_to_quat(rpy, point_to_local.rot_quat);
  
  BotTrans point_to_cam; 
  

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


    point_to_local.trans_vec[0] = self->ground_projections_local[i].x;
    point_to_local.trans_vec[1] = self->ground_projections_local[i].y;
    
    bot_trans_apply_trans_to(&local_to_cam, &point_to_local, &point_to_cam);
    
    //fprintf(stderr, "Dist : %f, %f - %f\n", point_to_cam.trans_vec[0], point_to_cam.trans_vec[1], hypot(point_to_cam.trans_vec[0], point_to_cam.trans_vec[1]));
    
    double dist_rat = fmin(1,hypot(point_to_cam.trans_vec[0], point_to_cam.trans_vec[1])/ 5.0);
    float *color = bot_color_util_jet(dist_rat);

    //self->ground_projections_local[i].x, self->ground_projections_local[i].y
    

    bot_lcmgl_color3f(lcmgl, color[0], color[1], color[2]);//r/255.0, g/255.0, b/255.0);
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

    //setup the vehicle mask 
     cv::Size size(msg->width, msg->height);
     state->vehicle_mask = cv::Mat::zeros(size, CV_8U);
     const int vm_count = (int)state->vehicle_mask_points.size();
     const cv::Point* vm_pts = &state->vehicle_mask_points[0];//&hull[0];
     cv::fillPoly(state->vehicle_mask, &vm_pts, &vm_count, 1, cv::Scalar(255));
     if(state->options.vDEBUG)
       cv::imshow("Vehicle Mask", state->vehicle_mask);
  }  

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

  //This routine does most of the heavy lifting 
  int64_t s_utime = bot_timestamp_now();
  int status = detect_road(state, msg->utime, state->img, state->hsv_img);      
  int64_t e_utime = bot_timestamp_now();
  if(status <0){
    fprintf(stderr, "Error Detecting road\n");
  }

  fprintf(stderr, "Time to process : %f\n",   (e_utime - s_utime)/1.0e6);

  state->img_utime = msg->utime; 
  cv::Mat display = state->img.clone();  
  cv::imshow("Camera", display);

  return;
}

//not used right now 
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
      g_mutex_unlock(self->mutex);
    }
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

  //setup the vehicle mask
  state->vehicle_mask_points.push_back(cv::Point(0, 124));
  state->vehicle_mask_points.push_back(cv::Point(0, 372));
  state->vehicle_mask_points.push_back(cv::Point(127, 543));
  state->vehicle_mask_points.push_back(cv::Point(228, 543));

  char * cam_name = bot_param_get_camera_name_from_lcm_channel(state->param,  state->options.vCHANNEL.c_str());
  if (cam_name != NULL) {
    state->camtrans = bot_param_get_new_camtrans(state->param, cam_name);
    state->coord_frame = bot_param_get_camera_coord_frame(state->param, cam_name);
 
    free(cam_name);
  }

  bot_core_image_t_subscribe(state->lcm, state->options.vCHANNEL.c_str(), on_image, state);

  //  pthread_create(&state->process_thread , NULL, p_thread, state);

  // Main lcm handle
  while(1) { 
    unsigned char c = cv::waitKey(1) & 0xff;
    lcm_handle(state->lcm);

    if (c == 'q') {
      break;  
    } else if ( c == 'c' ) {      
      if (state->options.vDEBUG) { 
	cv::imshow("Captured Image", state->img);
      }
    }
  }

  return 0;
  
}

