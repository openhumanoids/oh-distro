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

#define OFFSET_LARGE 5
#define OFFSET_SMALL 3
#define COSTMAP_RESOLUTION 0.1

using namespace std;
using namespace occ_map;

struct RoadDetectorOptions { 
    bool vDEBUG;
    std::string vCHANNEL;
    bool vSILENT;
    RoadDetectorOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vDEBUG(false), vSILENT(false){}
};

typedef struct _ImageVertex ImageVertex;
struct _ImageVertex {
    int tx;
    int ty;
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

void create_contour_pixelmap(Terrain *self, int64_t utime, cv::Size img_size, const vector< vector<cv::Point> > & obstacles, vector<cv::Point>contour, bot_lcmgl_t *lcmgl = NULL);

void create_contour_pixelmap(Terrain *self, int64_t utime, cv::Mat& mask, const vector< vector<cv::Point> > & obstacles, vector<cv::Point>contour, bot_lcmgl_t *lcmgl = NULL);

void dilate_mask(cv::Mat1b& mask, cv::Mat1b& mask_new, int offset=OFFSET_LARGE){

    //cvFindContours
    //offset 3 is good to remove just basic noise 
  
    //OFFSET 5 is good to remove the centerline 
    //const int OFFSET = 5;
    cv::dilate(mask, mask_new, cv::Mat(), cv::Point(-1,-1), offset);

    if(0){
        cv::Mat1b sm_display = mask_new.clone();
        cv::imshow("Dilated Mask", sm_display);
    }
}

static bool contour_size_compare(const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) { 
    return lhs.second < rhs.second;
}

//#if 0
void find_largest_contour_and_childern(cv::Mat1b& _mask, 
                                       std::vector<cv::Point>& largest_contour, 
                                       std::vector<std::vector<cv::Point> >& inner_contours) { 

    // find hierarchical contours
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::Size size(_mask.cols, _mask.rows);

    cv::findContours(_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); 

    cv::Mat1b filled_contour = cv::Mat::zeros(size, CV_8U);

    std::vector<std::pair<int, int> > contour_sizes; 
    for (int idx=0; idx>=0; idx = hierarchy[idx][0]) { 
        cv::Mat zmat = filled_contour.clone();
        drawContours(zmat, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy); 
        contour_sizes.push_back(std::make_pair(idx, cv::countNonZero(zmat)));
    }

    std::vector<std::pair<int, int> >::iterator it = 
        std::max_element(contour_sizes.begin(), contour_sizes.end(), contour_size_compare);
    int largest_contour_id = it->first;

    largest_contour = contours[largest_contour_id];

    // std::cerr << "MAX ID : " << largest_contour_id << std::endl;
    
    if(0){
        cv::Mat zmat = filled_contour.clone();
        drawContours(zmat, contours, largest_contour_id, cv::Scalar(255), CV_FILLED, 8, hierarchy); 
        cv::Mat3b zmat2 = cv::Mat3b::zeros(zmat.size());
        for (int idx=largest_contour_id; idx<largest_contour_id+1; idx++) { 
            std::vector<cv::Point>& contourj = contours[idx]; 
            for (int j=0; j<contourj.size(); j++)
                cv::circle(zmat2, contourj[j], 1, cv::Scalar(0,255,0)); 
        }
        cv::imshow("zmat", zmat);
        cv::imshow("zmat2", zmat2);
    }

    // Find all contours with the largest component as their parent
    std::vector<int> contour_children; 
    for (int idx=0; idx<contours.size(); idx++) { 
        if (hierarchy[idx][3] == largest_contour_id)
            contour_children.push_back(idx);
    }

    for (int j=0; j<contour_children.size(); j++) { 
        int idx = contour_children[j];
        inner_contours.push_back(contours[idx]);
    }

 
    return;
}
//#endif
int find_contours(cv::Mat1b& _mask, cv::Mat1b& filled_contour, vector<cv::Point>&largest_contour){

    cv::Mat1b mask = _mask.clone();
    cv::Size size(mask.cols, mask.rows);
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    // find contours and store them all as a list
    cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE); 

    //find largest contour
    int largest_countour_id = -1;
    int max_size = 0;

    for( size_t i = 0; i < contours.size(); i++){
        vector<cv::Point> c_contour = contours[i];
        if( max_size < c_contour.size()){
            max_size = c_contour.size();
            largest_countour_id = (int) i;
        }
    }  

    const cv::Scalar color(255,255,255);
  
    if(largest_countour_id >=0){
        largest_contour = contours[largest_countour_id];

        //we are not using convex hull anymore - it messes up when the road is turining
        if(0){
            cv::Mat1b mask_count = cv::Mat1b::zeros(mask.rows, mask.cols);
            std::vector<cv::Point> hull;
            cv::convexHull(largest_contour, hull);
      
            /*for(int j=1; j < largest_contour.size(); j++){
              cv::line(mask_count, largest_contour[j-1], largest_contour[j], color,  2);
              }*/
            for(int j=1; j < largest_contour.size(); j++){
                cv::circle(mask_count, largest_contour[j],3, color,  2);
            }
        
            cv::imshow("Contour Points", mask_count);

            /*cv::Mat hull_contour = cv::Mat::zeros(size, CV_8U);
              const int hull_count = (int)hull.size();
              const cv::Point* hull_pts = &hull[0];//&hull[0];
              cv::fillPoly(hull_contour, &hull_pts, &hull_count, 1, cv::Scalar(255));
      
              cv::imshow("Hull Mask", hull_contour);*/
        }

        filled_contour = cv::Mat::zeros(size, CV_8U);
        const int filled_count = (int)largest_contour.size();
        const cv::Point* filled_pts = &largest_contour[0];//&hull[0];
        cv::fillPoly(filled_contour, &filled_pts, &filled_count, 1, cv::Scalar(255));
    
        return 0;
    }
    else{
        fprintf(stderr, "Error - No contour found\n");
        return -1;
    }  
}

//for this get all the contour points - otherwise it will miss some obstacles
void find_all_contours(cv::Mat1b& _mask, vector<vector<cv::Point> >& contours){
    cv::Mat1b mask = _mask.clone();
    // find contours and store them all as a list
    cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
}

void find_all_contours(cv::Mat& _mask, vector<vector<cv::Point> >& contours){
    cv::Mat1b mask = _mask.clone();
    // find contours and store them all as a list
    cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
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

    if(self->options.vDEBUG){
        //display the HSV results 
        cv::Mat h_display = hue.clone();
        cv::imshow("HUE", h_display);

        cv::Mat v_display = val.clone();
        cv::imshow("VAL", v_display);

        cv::Mat s_display = sat.clone();
        cv::imshow("SAT", s_display);
    }

    //we might need another filtering part for the yellow road lines
    cv::Mat1b mask = (self->vehicle_mask == 0 
                      & ((val <= 255 * 0.05 & sat > 255* 0.1 & sat < 255 * 0.4)) 
                      );


    //the dilation is need for the contour to work - otherwise it will pick only a part of the road sometimes 
    cv::Mat1b mask_dil;
    dilate_mask(mask, mask_dil, OFFSET_LARGE);

    //get the contour points and the contour 
    cv::Mat1b filled_contour_l;
    vector<cv::Point> largest_contour_l;
    int status = find_contours(mask_dil, filled_contour_l, largest_contour_l);

    if(status <0){
        fprintf(stderr, "Error : Dilated Contour - Failed to find large contour\n");
        return -3;
    }
    

    //detect obstacles 
    cv::Mat1b mask_dil_small;
    dilate_mask(mask, mask_dil_small, 2);

    cv::Mat1b road_paint = (mask_dil_small == 0 & filled_contour_l >0 & (hue <30 & hue > 15));

    cv::Mat1b road_paint_dil;
    dilate_mask(road_paint, road_paint_dil, 2);

    if(!self->options.vSILENT)
        cv::imshow("Road Paint (Small Dilation)" , road_paint_dil);
    //maybe dilate the road paint a bit 

    cv::Mat1b road_and_paint = (mask_dil_small | road_paint_dil);
      
    if(!self->options.vSILENT)
        cv::imshow("Road with Paint", road_and_paint);

    vector<cv::Point> largest_contour;
    vector< vector<cv::Point> > obs_contours;

    find_largest_contour_and_childern(road_and_paint, 
                                      largest_contour, 
                                      obs_contours);

    fprintf(stderr, "Largest Contour Size : %d\n", largest_contour.size());

    cv::Size size(road_and_paint.cols, road_and_paint.rows);
    
    if(largest_contour.size() >0){
        //we can prob not do this also 

        //the road outine in the image space is projected to ground plane and then used to calculate 
        //a distance map - from road edges - and published for use by the controller
        create_contour_pixelmap(self, utime, size, obs_contours, largest_contour, self->lcmgl);
        //delete contour_map->data;
        //contour_map->data = NULL;
        //delete contour_map;
    }  
    else{
        //this mask works 
        fprintf(stderr, "Error - Did not find largest contour - this should not have reached - should exit early");
        //project_to_ground(self, utime, img, mask, self->lcmgl);
    }

    cv::Mat1b sm_display = mask.clone();
    if(!self->options.vSILENT)
        cv::imshow("Basic Road Detection", sm_display);
  
    return 0;
}

int detect_road_old(Terrain *self, int64_t utime, cv::Mat& img, cv::Mat &hsv_img)
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

    if(self->options.vDEBUG){
        //display the HSV results 
        cv::Mat h_display = hue.clone();
        cv::imshow("HUE", h_display);

        cv::Mat v_display = val.clone();
        cv::imshow("VAL", v_display);

        cv::Mat s_display = sat.clone();
        cv::imshow("SAT", s_display);
    }

    //we might need another filtering part for the yellow road lines
    cv::Mat1b mask = (self->vehicle_mask == 0 
                      & ((val <= 255 * 0.05 & sat > 255* 0.1 & sat < 255 * 0.4)) 
                      //(hue  35 & hue < 50))
                      );

    //this is a simple hsv filter to detect road paint - sometimes gets the top of the traffic cone - but i dont think its a problem 
    /*cv::Mat1b road_lines = (self->vehicle_mask == 0 
      & (hue <30 & hue > 15));
    */
    // & val > 80); // & sat > 225 & val < 180 & val > 130));
    //& (hue > 35 & hue < 50  & val > 200)
			    
    //cv::imshow("Road Paint", road_lines);

    //the dilation is need for the contour to work - otherwise it will pick only a part of the road sometimes 
    cv::Mat1b mask_dil;
    dilate_mask(mask, mask_dil, OFFSET_LARGE);

    //get the contour points and the contour 
    cv::Mat1b filled_contour_l;
    vector<cv::Point> largest_contour_l;
    int status = find_contours(mask_dil, filled_contour_l, largest_contour_l);

    if(status <0){
        fprintf(stderr, "Error : Failed to find large contour\n");
        return -3;
    }

    //detect obstacles 
    cv::Mat1b mask_dil_small;
    dilate_mask(mask, mask_dil_small, 2);

    cv::Mat1b road_paint = (mask_dil_small == 0 & filled_contour_l >0 & (hue <30 & hue > 15))
        ;
    cv::Mat1b road_paint_dil;
    dilate_mask(road_paint, road_paint_dil, 2);

    cv::imshow("Road Paint (Small Dilation)" , road_paint_dil);
    //maybe dilate the road paint a bit 

    cv::Mat1b road_and_paint = (mask_dil_small | road_paint_dil);
  
    cv::imshow("Road with Paint", road_and_paint);

    //get the contour points and the contour 
    cv::Mat1b filled_contour;
    vector<cv::Point> largest_contour;
    cv::Mat1b road_and_paint_c = road_and_paint.clone();
    status = find_contours(road_and_paint_c, filled_contour, largest_contour);

    //contour blurs the obstacles (sometimes) - depends on the position
    //we will have to add the ones we missed - due to the contour 
    cv::Size size(mask.cols, mask.rows);

    cv::Mat1b obstacles = (road_and_paint == 0 & filled_contour == 255);

    //get the contours for the obstacles - we will just pass that in to the function
    vector< vector<cv::Point> > obs_contours;
    //find contours seems to mess with the data in obstacles 
    find_all_contours(obstacles, obs_contours);

    cv::imshow("Obstacles", obstacles);
  
    if(!filled_contour.empty()){
        //we can prob not do this also 
        cv::imshow("Contour Fill", filled_contour.clone());
        cv::Mat1b mask_filled = (filled_contour > 0);            
    
        //the road outine in the image space is projected to ground plane and then used to calculate 
        //a distance map - from road edges - and published for use by the controller
        create_contour_pixelmap(self, utime, mask_filled, obs_contours, largest_contour, self->lcmgl);
        //delete contour_map->data;
        //contour_map->data = NULL;
        //delete contour_map;
    }  
    else{
        //this mask works 
        fprintf(stderr, "Error - Did not find largest contour - this should not have reached - should exit early");
        //project_to_ground(self, utime, img, mask, self->lcmgl);
    }

    cv::Mat1b sm_display = mask.clone();
    cv::imshow("Basic Road Detection", sm_display);
  
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

//set up the rays from the camera (applying corrections etc)
void setup_projection(Terrain *self, int width, int height){//cv::Mat road){
    BotTrans cam_to_body;
    bot_frames_get_trans(self->frames, self->coord_frame, "body", //bot_frames_get_root_name(self->frames),
                         &cam_to_body);

    int xstep = 1;
    int ystep = 1;
    int ncols = width;
    int nrows = height;
  
    self->img_nvertices = ncols * nrows;
    int img_data_size = self->img_nvertices * sizeof(ImageVertex);
    self->vertices = (ImageVertex*) malloc(img_data_size);
    //self->n_vert_indices = (ncols - 1) * (nrows - 1) * 4;
    //self->vert_indices = (int*) malloc(self->n_vert_indices * sizeof(int));
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
        }
    }
}

//project the imagespace road contours on to the ground plane - and then do a distance transform - which is sent off to the controller
void create_contour_pixelmap(Terrain *self, int64_t utime, cv::Mat& mask, const vector< vector<cv::Point> >& obstacles, vector<cv::Point>contour, bot_lcmgl_t *lcmgl){
    
    BotTrans head_to_local;

    bot_frames_get_trans_with_utime(self->frames, "head", bot_frames_get_root_name(self->frames),
                                    utime, &head_to_local);
  
    BotTrans cam_to_local;

    bot_frames_get_trans_with_utime(self->frames, self->coord_frame, bot_frames_get_root_name(self->frames),
                                    utime, &cam_to_local);

    //fprintf(stderr, "Done with Pixel Map - No of Cells : %d\n", distance_map->num_cells);
    //We need a transform from the Cam to center of Vehicle - which is where we are trying to drive from 
  
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
  
    corner_to_head.trans_vec[0] = 25;
    corner_to_head.trans_vec[1] = -10;
  
    bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

    min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
    min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

    max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
    max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

    corner_to_head.trans_vec[0] = 25;
    corner_to_head.trans_vec[1] = 10;
  
    bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

    min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
    min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

    max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
    max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

    corner_to_head.trans_vec[0] = 0;
    corner_to_head.trans_vec[1] = 10;
  
    bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

    min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
    min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

    max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
    max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

    if(lcmgl){
        bot_lcmgl_point_size(lcmgl, 5);
        bot_lcmgl_begin(lcmgl, GL_POINTS);
    }

    double resolution = COSTMAP_RESOLUTION;
  
    PixelMap<float>* distance_map = new PixelMap<float>(min_xy, max_xy, resolution, false, false);
    distance_map->data = new float[distance_map->num_cells];
    memset(distance_map->data, 0, sizeof(float) * distance_map->num_cells);

    cv::Mat1b projected_map = cv::Mat1b::zeros(distance_map->dimensions[1], distance_map->dimensions[0]);
    cv::Mat1b filled_map = cv::Mat1b::zeros(distance_map->dimensions[1], distance_map->dimensions[0]);
    cv::Mat1b filled_obs_map = cv::Mat1b::zeros(distance_map->dimensions[1], distance_map->dimensions[0]);
  
    BotTrans local_to_cam = cam_to_local;
    bot_trans_invert(&local_to_cam);
  
    BotTrans point_to_local;
    point_to_local.trans_vec[2] = 0;
    bot_roll_pitch_yaw_to_quat(rpy, point_to_local.rot_quat);
  
    BotTrans point_to_cam; 
    int skip = 1;
    // project image onto the ground plane
    int map_count = 0;

    int last_valid_ind = 0;

    vector<cv::Point> transformed_contour_points;
    cv::Mat1b mask_count = cv::Mat1b::zeros(mask.rows, mask.cols);
    const cv::Scalar color_1(255,255,255);

    for(size_t i=0; i < contour.size(); i++){
        cv::Point pt = contour[i];
    
        int ind = pt.x + pt.y * mask.cols;
    
        cv::circle(mask_count, pt ,3, color_1,  2);
    
        ImageVertex *v = &self->vertices[ind];
    
        point3d_t pixel_rays_local;
        pixel_rays_local.x = v->vx;
        pixel_rays_local.y = v->vy;
        pixel_rays_local.z = v->vz;
    
        point2d_t ground_projections_local;
    
        double v_cam[3] = { v->vx, v->vy, v->vz };
    
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
    
        bot_trans_apply_trans_to(&local_to_cam, &point_to_local, &point_to_cam);
    
        double xy[2] = {ground_projections_local.x, ground_projections_local.y};
    
        int ixy[2];
    
        if(distance_map->isInMap(xy)){
            distance_map->worldToTable(xy, ixy);
            transformed_contour_points.push_back(cv::Point(ixy[0], ixy[1]));
      
            if(lcmgl){
                bot_lcmgl_color3f(lcmgl, 1.0, 0, 0);
                bot_lcmgl_vertex3f(lcmgl, ground_projections_local.x, ground_projections_local.y, 0);
            }
        }
    }

    vector< vector<cv::Point> > transformed_obstacles;
  
    for(size_t i=0; i < obstacles.size(); i++){
        vector<cv::Point> obs = obstacles[i];
    
        vector<cv::Point> tf_obs_points; 
        for(size_t j=0; j < obs.size(); j++){
      
            cv::Point pt = obs[j];
    
            int ind = pt.x + pt.y * mask.cols;
    
            cv::circle(mask_count, pt ,3, color_1,  2);
    
            ImageVertex *v = &self->vertices[ind];
    
            point3d_t pixel_rays_local;
            pixel_rays_local.x = v->vx;
            pixel_rays_local.y = v->vy;
            pixel_rays_local.z = v->vz;
    
            point2d_t ground_projections_local;
    
            double v_cam[3] = { v->vx, v->vy, v->vz };
    
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
    
            bot_trans_apply_trans_to(&local_to_cam, &point_to_local, &point_to_cam);
    
            double xy[2] = {ground_projections_local.x, ground_projections_local.y};
    
            int ixy[2];
    
            if(distance_map->isInMap(xy)){
                distance_map->worldToTable(xy, ixy);
                tf_obs_points.push_back(cv::Point(ixy[0], ixy[1]));

                if(lcmgl){
                    bot_lcmgl_color3f(lcmgl, 1.0, 0, 0);
                    bot_lcmgl_vertex3f(lcmgl, ground_projections_local.x, ground_projections_local.y, 0);
                }
            }
        }
        if(tf_obs_points.size() > 0){
            transformed_obstacles.push_back(tf_obs_points);
        }
    }

  
    //get the obstacle masks 
    if(transformed_obstacles.size() > 0){
        for(int i=0; i< transformed_obstacles.size(); i++){
            vector<cv::Point> obs = transformed_obstacles[i];
            fprintf(stderr, "Adding Points : %d\n", (int)obs.size()); 
            const int filled_count = (int)obs.size();
            const cv::Point* filled_pts = &obs[0];
            cv::fillPoly(filled_obs_map, &filled_pts, &filled_count, 1, cv::Scalar(255));
        }
    }

    //cv::Mat1b obs_free_map = (!filled_obs_map);

    cv::imshow("Contour Points Projected", mask_count);
  
    const int filled_count = (int)transformed_contour_points.size();
    const cv::Point* filled_pts = &transformed_contour_points[0];
    cv::fillPoly(filled_map, &filled_pts, &filled_count, 1, cv::Scalar(255));
  
    cv::Mat1b road_map = (filled_obs_map==0 & filled_map);

    imshow("Filled Transformed Contour", filled_map);

    imshow("Transformed Road Map", road_map);

    cv::Mat1b road_outline = (road_map >0);//(filled_map >0);

    //we should do this - but to the projected 
    /*cv::Mat1b road_outline = (mask >0);//cv::Mat1b::zeros(mask.rows, mask.cols);
      const cv::Scalar color(255,255,255);  
      for(int j=1; j < contour.size(); j++){
      cv::line(road_outline, contour[j-1], contour[j], color,  2);
      }

      for(int i= 0; i < road_outline.cols; i++){
      for(int j= road_outline.rows - 20; j < road_outline.rows; j++){
      road_outline.at<uint8_t>(j,i) = 1;
      }
      }*/
  
    //give up the bottom set of pixels - because we see the car sometimes 
  
    cv::Size size(road_outline.cols, road_outline.rows);
    cv::Mat dist_color = cv::Mat::zeros(size, CV_8U);

    cv::Mat dist;
    distanceTransform(road_outline, dist, CV_DIST_L2, CV_DIST_MASK_5); //CV_DIST_MASK_PRECISE);

    dist *= resolution;
  
    if(0){
        uint8_t max = 0;
        for(int i=0; i < dist.cols; i++){
            for(int j=0; j < dist.rows; j++){
                uint8_t d = 254* fmin(7, dist.at<float>(j,i))/7.0;
                dist_color.at<uint8_t>(j,i) = d;
                if(d > max)
                    max = d;
            }
        }
        cv::imshow("Road Distance Transform", dist_color);
    }
    
    for(int i=0; i < dist.cols; i++){
        for(int j=0; j < dist.rows; j++){
            float d = dist.at<float>(j,i);
            int ixy[2] = {i,j};
            distance_map->writeValue(ixy, d);
        }
    }

    const occ_map_pixel_map_t *map_msg = distance_map->get_pixel_map_t(utime);
    occ_map_pixel_map_t_publish(self->lcm, "TERRAIN_DIST_MAP", map_msg);   

    delete distance_map->data;
    distance_map->data = NULL;
    delete distance_map;

    if(lcmgl){
        bot_lcmgl_end(lcmgl);
        bot_lcmgl_switch_buffer(lcmgl);
    }  
}

//project the imagespace road contours on to the ground plane - and then do a distance transform - which is sent off to the controller
void create_contour_pixelmap(Terrain *self, int64_t utime, cv::Size img_size, const vector< vector<cv::Point> >& obstacles, vector<cv::Point>contour, bot_lcmgl_t *lcmgl){
    
    BotTrans head_to_local;

    bot_frames_get_trans_with_utime(self->frames, "head", bot_frames_get_root_name(self->frames),
                                    utime, &head_to_local);
  
    BotTrans cam_to_local;

    bot_frames_get_trans_with_utime(self->frames, self->coord_frame, bot_frames_get_root_name(self->frames),
                                    utime, &cam_to_local);

    //fprintf(stderr, "Done with Pixel Map - No of Cells : %d\n", distance_map->num_cells);
    //We need a transform from the Cam to center of Vehicle - which is where we are trying to drive from 
  
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
  
    corner_to_head.trans_vec[0] = 25;
    corner_to_head.trans_vec[1] = -10;
  
    bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

    min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
    min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

    max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
    max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

    corner_to_head.trans_vec[0] = 25;
    corner_to_head.trans_vec[1] = 10;
  
    bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

    min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
    min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

    max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
    max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

    corner_to_head.trans_vec[0] = 0;
    corner_to_head.trans_vec[1] = 10;
  
    bot_trans_apply_trans_to(&head_to_local, &corner_to_head, &corner_to_local);

    min_xy[0] = fmin(corner_to_local.trans_vec[0], min_xy[0]);
    min_xy[1] = fmin(corner_to_local.trans_vec[1], min_xy[1]);

    max_xy[0] = fmax(corner_to_local.trans_vec[0], max_xy[0]);
    max_xy[1] = fmax(corner_to_local.trans_vec[1], max_xy[1]);

    if(lcmgl){
        bot_lcmgl_point_size(lcmgl, 5);
        bot_lcmgl_begin(lcmgl, GL_POINTS);
    }

    double resolution = COSTMAP_RESOLUTION;
  
    PixelMap<float>* distance_map = new PixelMap<float>(min_xy, max_xy, resolution, false, false);
    distance_map->data = new float[distance_map->num_cells];
    memset(distance_map->data, 0, sizeof(float) * distance_map->num_cells);

    cv::Mat1b projected_map = cv::Mat1b::zeros(distance_map->dimensions[1], distance_map->dimensions[0]);
    cv::Mat1b filled_map = cv::Mat1b::zeros(distance_map->dimensions[1], distance_map->dimensions[0]);
    cv::Mat1b filled_obs_map = cv::Mat1b::zeros(distance_map->dimensions[1], distance_map->dimensions[0]);
  
    BotTrans local_to_cam = cam_to_local;
    bot_trans_invert(&local_to_cam);
  
    BotTrans point_to_local;
    point_to_local.trans_vec[2] = 0;
    bot_roll_pitch_yaw_to_quat(rpy, point_to_local.rot_quat);
  
    BotTrans point_to_cam; 
    int skip = 1;
    // project image onto the ground plane
    int map_count = 0;

    int last_valid_ind = 0;

    vector<cv::Point> transformed_contour_points;
    //cv::Mat1b mask_count = cv::Mat1b::zeros(img_size);
    
    cv::Mat mask_count = cv::Mat3b::zeros(img_size);//, CV_8U);
    const cv::Scalar color_1(255,255,255);
    const cv::Scalar color_obs(0,0,255);

    for(size_t i=0; i < contour.size(); i++){
        cv::Point pt = contour[i];
    
        int ind = pt.x + pt.y * img_size.width;
    
        cv::circle(mask_count, pt ,3, color_1,  2);
    
        ImageVertex *v = &self->vertices[ind];
    
        point3d_t pixel_rays_local;
        pixel_rays_local.x = v->vx;
        pixel_rays_local.y = v->vy;
        pixel_rays_local.z = v->vz;
    
        point2d_t ground_projections_local;
    
        double v_cam[3] = { v->vx, v->vy, v->vz };
    
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
    
        bot_trans_apply_trans_to(&local_to_cam, &point_to_local, &point_to_cam);
    
        double xy[2] = {ground_projections_local.x, ground_projections_local.y};
    
        int ixy[2];
    
        if(distance_map->isInMap(xy)){
            distance_map->worldToTable(xy, ixy);
            transformed_contour_points.push_back(cv::Point(ixy[0], ixy[1]));
      
            if(lcmgl){
                bot_lcmgl_color3f(lcmgl, 1.0, 0, 0);
                bot_lcmgl_vertex3f(lcmgl, ground_projections_local.x, ground_projections_local.y, 0);
            }
        }
    }

    vector< vector<cv::Point> > transformed_obstacles;
  
    for(size_t i=0; i < obstacles.size(); i++){
        vector<cv::Point> obs = obstacles[i];
        //fprintf(stderr, "Size of obs : %d\n", obs.size());
        //we might want to skip these objects - if they are small??
        vector<cv::Point> tf_obs_points; 
        for(size_t j=0; j < obs.size(); j++){
      
            cv::Point pt = obs[j];
    
            int ind = pt.x + pt.y * img_size.width;
    
            cv::circle(mask_count, pt ,3, color_obs,  2);
    
            ImageVertex *v = &self->vertices[ind];
    
            point3d_t pixel_rays_local;
            pixel_rays_local.x = v->vx;
            pixel_rays_local.y = v->vy;
            pixel_rays_local.z = v->vz;
    
            point2d_t ground_projections_local;
    
            double v_cam[3] = { v->vx, v->vy, v->vz };
    
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
    
            bot_trans_apply_trans_to(&local_to_cam, &point_to_local, &point_to_cam);
    
            double xy[2] = {ground_projections_local.x, ground_projections_local.y};
    
            int ixy[2];
    
            if(distance_map->isInMap(xy)){
                distance_map->worldToTable(xy, ixy);
                tf_obs_points.push_back(cv::Point(ixy[0], ixy[1]));

                if(lcmgl){
                    bot_lcmgl_color3f(lcmgl, 1.0, 0, 0);
                    bot_lcmgl_vertex3f(lcmgl, ground_projections_local.x, ground_projections_local.y, 0);
                }
            }
        }
        if(tf_obs_points.size() > 0){
            transformed_obstacles.push_back(tf_obs_points);
        }
    }

  
    //get the obstacle masks 
    if(transformed_obstacles.size() > 0){
        for(int i=0; i< transformed_obstacles.size(); i++){
            vector<cv::Point> obs = transformed_obstacles[i];
            fprintf(stderr, "Adding Points : %d\n", (int)obs.size()); 
            const int filled_count = (int)obs.size();
            const cv::Point* filled_pts = &obs[0];
            cv::fillPoly(filled_obs_map, &filled_pts, &filled_count, 1, cv::Scalar(255));
        }
    }

    cv::imshow("Contour Points (Img Space)", mask_count);
  
    const int filled_count = (int)transformed_contour_points.size();
    const cv::Point* filled_pts = &transformed_contour_points[0];
    cv::fillPoly(filled_map, &filled_pts, &filled_count, 1, cv::Scalar(255));
  
    cv::Mat1b road_map = (filled_obs_map==0 & filled_map);

    if(!self->options.vSILENT)
        imshow("Filled Transformed Contour", filled_map);
    
    imshow("Transformed Road Map", road_map);

    cv::Mat1b road_outline = (road_map >0);//(filled_map >0);

    //we should do this - but to the projected 
    /*cv::Mat1b road_outline = (mask >0);//cv::Mat1b::zeros(mask.rows, mask.cols);
      const cv::Scalar color(255,255,255);  
      for(int j=1; j < contour.size(); j++){
      cv::line(road_outline, contour[j-1], contour[j], color,  2);
      }

      for(int i= 0; i < road_outline.cols; i++){
      for(int j= road_outline.rows - 20; j < road_outline.rows; j++){
      road_outline.at<uint8_t>(j,i) = 1;
      }
      }*/
  
    //give up the bottom set of pixels - because we see the car sometimes 
  
    cv::Size size(road_outline.cols, road_outline.rows);
    cv::Mat dist_color = cv::Mat::zeros(size, CV_8U);

    cv::Mat dist;
    distanceTransform(road_outline, dist, CV_DIST_L2, CV_DIST_MASK_5); //CV_DIST_MASK_PRECISE);

    dist *= resolution;
  
    if(0){
        uint8_t max = 0;
        for(int i=0; i < dist.cols; i++){
            for(int j=0; j < dist.rows; j++){
                uint8_t d = 254* fmin(7, dist.at<float>(j,i))/7.0;
                dist_color.at<uint8_t>(j,i) = d;
                if(d > max)
                    max = d;
            }
        }
        cv::imshow("Road Distance Transform", dist_color);
    }
    
    for(int i=0; i < dist.cols; i++){
        for(int j=0; j < dist.rows; j++){
            float d = dist.at<float>(j,i);
            int ixy[2] = {i,j};
            distance_map->writeValue(ixy, d);
        }
    }

    const occ_map_pixel_map_t *map_msg = distance_map->get_pixel_map_t(utime);
    occ_map_pixel_map_t_publish(self->lcm, "TERRAIN_DIST_MAP", map_msg);   

    delete distance_map->data;
    distance_map->data = NULL;
    delete distance_map;

    if(lcmgl){
        bot_lcmgl_end(lcmgl);
        bot_lcmgl_switch_buffer(lcmgl);
    }  
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
    opt.add(state->options.vSILENT, "s", "silent","Silent mode");
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

