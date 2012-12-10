// mfallon sept 2012

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <getopt.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <path_util/path_util.h>
#include <lcmtypes/bot_param_update_t.h>

#include <ConciseArgs>
#include "heightmap_lcmgl.hpp"

using namespace std;
using namespace Eigen;

bool VERBOSE =false;

heightmap_lcmgl::heightmap_lcmgl(lcm_t* publish_lcm,lcm_t* subscribe_lcm):
          publish_lcm_(publish_lcm),
          subscribe_lcm_(subscribe_lcm){

  lcmgl_ = bot_lcmgl_init(publish_lcm, "drive-map");
	    
  cloud_counter =0;
  rgb_buf_size_ = 640 * 480;
  rgb_buf_ = (uint8_t*) malloc(rgb_buf_size_* 3);
}

// requires #include <limits>
template<typename T>
inline bool isinf_heightmap(T value)
{
return std::numeric_limits<T>::has_infinity &&
value == std::numeric_limits<T>::infinity();
}

void projectCorners(Eigen::Isometry3d pose){
    double fx = 320; // focal length at of nov 2012
    double width = 640;
    double height = 480;  
    Vector3d ptQ(1,  width/(2*fx) , - height/(2*fx) );
    // x and y negated
  
    Eigen::Quaterniond quat(pose.rotation());
    
  Eigen::Translation3d translation ( pose.translation() );
  // Assemble an Eigen Transform
  Eigen::Affine3d transform;
  transform = translation * quat;
    
  cout << "\n================\npre: " << ptQ << "\n\n";
  ptQ=    transform*ptQ;
  cout << "pos: " << ptQ<< "\n\n";
  
  Vector3d ptP (0,0,0);
  ptP << pose.translation(); //( ,2,1); // Cam point
//  Vector3d ptQ ( 1,2.1,0.1);

  // Intersect the point with the ground:
  cout << ptP  << " is the point\n";
  double t = -ptP(2)/( ptQ(2) - ptP(2));
  Vector3d ptZ ( 0,0,0);
  ptZ(0) = ptP(0) + t*(ptQ(0) - ptP(0));
  ptZ(1) = ptP(1) + t*(ptQ(1) - ptP(1));
  cout << ptZ  << " is the ground point\n";  
}


void heightmap_lcmgl::image_handler(const bot_core_image_t *msg){
  cout << "got image\n";

  std::cout << msg->utime << "\n";

  // input: focal length
  // input: camera pose
  // 1. determine 3d position of corner points
  // 2. transform these points using the camera pose
  // 3. determine line containing focal position and the points
  // 4. intesect line with z=0
  
  //Vector equation : P + t (Q-P)
  // t = -Pz/(Qz-Pz)  // intersection of line with ground
  
        Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation() << 0,0,1;
  Eigen::Quaterniond m;
  m  = Eigen::Quaterniond(1,0,0,0);
  pose.rotate(m);

  projectCorners(pose);
  
//  return;
   
  
    bot_lcmgl_push_matrix(lcmgl_);
//    bot_lcmgl_translated(lcmgl_, x_offset, 2*base_height+10, 0);
  
  // current image
  bot_lcmgl_color3f(lcmgl_, 1, 1, 1);
  const uint8_t* target_gray = msg->data;
  cout << msg->height << " " << msg->width << " " <<  msg->row_stride << "\n";
  cout << msg->pixelformat << " is format\n";
  int height = msg->height;
  int width = msg->width;
  int row_stride = msg->row_stride;
  
  int gray_texid = bot_lcmgl_texture2d(lcmgl_, target_gray, width, height,
                                       row_stride, BOT_LCMGL_RGB, // grey:BOT_LCMGL_LUMINANCE,
                                       BOT_LCMGL_UNSIGNED_BYTE,
                                       BOT_LCMGL_COMPRESS_NONE);
  bot_lcmgl_texture_draw_quad(lcmgl_, gray_texid,
      0     , 0      , 0   ,
      0     , height , 0   ,
      width , height , 0   ,
      width , 0      , 0);  
 
      bot_lcmgl_pop_matrix(lcmgl_);  

  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_rotated(lcmgl_, -90, 0, 0, 1);
  bot_lcmgl_rotated(lcmgl_, -90, 1, 0, 0);

//  bot_lcmgl_translated(_lcmgl, 0, pose.translation().y(), 0);
  bot_lcmgl_color3f(lcmgl_, 0, 1, 1);
  bot_lcmgl_point_size(lcmgl_, 1.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);

 
//  if (status==0){
    bot_lcmgl_color3f(lcmgl_, 0, 0, 1);
//  }else{ // status 1 when changing key frames:
//    bot_lcmgl_color3f(lcmgl_, 1, 0, 0);
//  }
  for (size_t i=0; i < 100; ++i) {

    bot_lcmgl_vertex3f(lcmgl_, i,  i, i);
//    bot_lcmgl_vertex3f(lcmgl_, f.xyz[2],  -f.xyz[0],-f.xyz[1]);
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);
  bot_lcmgl_switch_buffer(lcmgl_);  
  
}


void heightmap_lcmgl::heightmap_handler(const drc_heightmap_t *msg){
  cout << "got hm: " << msg->utime << "\n";
  
      // write pgm height map (for debugging)
      std::ofstream ofs("heightmap.txt");
    for (int i=0; i < msg->ny;i++){
    for (int j=0; j < msg->nx;j++){
      int index = i*msg->nx + j;
          ofs << msg->heights[index] << " ";
        }
        ofs << std::endl;
      }
      std::cout << "Wrote debug heightmap.txt" << std::endl;
  
  
  
  Eigen::Isometry3f mTransformToLocal;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mTransformToLocal(i,j) = ((float)	  msg->transform_to_local[i][j]);
    }
  }  
  
  cout << "\n";
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      cout << mTransformToLocal(i,j) << ", ";
    }
    cout << "\n";
  }  
  //mTransformToLocal(0,3) += hack_offset_x_;
  //mTransformToLocal(1,3) += hack_offset_y_;
  //mTransformToLocal(2,3) += hack_offset_z_;
  
  double pelvis_to_ground =1; // due to pinned pelvis from osrf
  
  
  if (1==0){  // just points
    bot_lcmgl_push_matrix(lcmgl_);
    bot_lcmgl_point_size(lcmgl_, 10.0f);
    bot_lcmgl_begin(lcmgl_, GL_POINTS);
    bot_lcmgl_color4f(lcmgl_, 0.0, 0.0, 1.0,0.5);
      for (int i = 0; i < msg->ny; i++) {
        for (int j = 0; j < msg->nx; j++) { //
	  Vector4f pt4(j,i, msg->heights[i*msg->nx+j], 1);
	  pt4= mTransformToLocal*pt4;
          if ( msg->heights[i*msg->nx+j] > 100){
	  }else{
             bot_lcmgl_vertex3f(lcmgl_, pt4(0), pt4(1),  pt4(2) + pelvis_to_ground);            
          }
        }
      }
    bot_lcmgl_end(lcmgl_);
    bot_lcmgl_pop_matrix(lcmgl_);
  }else{
    
    bot_lcmgl_push_matrix(lcmgl_);
    
    bot_lcmgl_point_size(lcmgl_, 5.0f);
    bot_lcmgl_begin(lcmgl_, GL_TRIANGLES);//GL_TRIANGLE_FAN);
    
      for (int i = 0; i < msg->ny; i++) {
        for (int j = 0; j < msg->nx; j++) { //
           int label=0; // red by default
           
           //cout << msg->heights[i*msg->nx+j] << " val\n";
           //if ( isinf_heightmap ( fabs(msg->heights[i*msg->nx+j])) == 0 ){
	   //  cout << "...... inf\n"; 
	   //} // this wont work..
	   //if ( msg->heights[i*msg->nx+j] < -99999999.0 ){
	   // cout << "...... inf\n"; 
	   //} // using this instead

	  float ptheight =  (float)  msg->heights[i*msg->nx+j] + pelvis_to_ground; // true height off of ground
	  if ( msg->heights[i*msg->nx+j] < -99999999.0 ){
	    label = 2; // gray/undef
	  }else if ((ptheight < 0.2)&&( ptheight > -0.2)){
	    label = 1; // good
	  }
	    
	  Vector4f ptA(  ((float) j - 0.5)   , ((float) i - 0.5), 0, 1.0);
	  ptA= mTransformToLocal*ptA;
	  Vector4f ptB(  ((float) j + 0.5)   , ((float)i - 0.5), 0, 1.0);
	  ptB= mTransformToLocal*ptB;
	  Vector4f ptC(  ((float) j + 0.5)   , ((float) i + 0.5), 0, 1.0);
	  ptC= mTransformToLocal*ptC;
	  Vector4f ptD(  ((float) j - 0.5)   , ((float)i + 0.5), 0, 1.0);
	  ptD= mTransformToLocal*ptD;

	  if (VERBOSE){
	    cout << "A: " << ptA(0) << ", " << ptA(1) <<", " << ptA(2) << "\n"; 
	    cout << "B: " << ptB(0) << ", " << ptB(1) <<", " << ptB(2) << "\n"; 
	    cout << "C: " << ptC(0) << ", " << ptC(1) <<", " << ptC(2) << "\n"; 
	    cout << "D: " << ptD(0) << ", " << ptD(1) <<", " << ptD(2) << "\n"; 
	  }
	  
	  if (label==0){ // red/occupied
	    bot_lcmgl_color4f(lcmgl_, 1.0, 0.8, 0.8,0.1);
	  }else if ( label==1 ){ // green good
	    bot_lcmgl_color4f(lcmgl_, 0.8, 1.0, 0.8,0.1);
	  }else if ( label==2 ){ // grey unknown
	    bot_lcmgl_color4f(lcmgl_, 0.8, 0.8, 0.8,0.1);
	  }
	  
          bot_lcmgl_vertex3f(lcmgl_, ptA(0), ptA(1), -pelvis_to_ground);//ptA(2));            
          bot_lcmgl_vertex3f(lcmgl_, ptB(0), ptB(1), -pelvis_to_ground);//ptB(2));            
          bot_lcmgl_vertex3f(lcmgl_, ptC(0), ptC(1), -pelvis_to_ground);//ptC(2));        
          bot_lcmgl_vertex3f(lcmgl_, ptA(0), ptA(1), -pelvis_to_ground);//ptA(2));            
          bot_lcmgl_vertex3f(lcmgl_, ptC(0), ptC(1), -pelvis_to_ground);//ptC(2));        
          bot_lcmgl_vertex3f(lcmgl_, ptD(0), ptD(1), -pelvis_to_ground);//ptD(2));  
        }
      }
    bot_lcmgl_end(lcmgl_);
    bot_lcmgl_pop_matrix(lcmgl_);
  }
  
  /*
      std::ofstream ofs("/home/mfallon/heightmap.txt");
      for (int i = 0; i < msg->nx; ++i) {
        for (int j = 0; j < heightMap.mWidth; ++j) {
          ofs << heightMap.mData[i*heightMap.mWidth+j] << " ";
        }
        ofs << std::endl;
      }
      std::cout << "Writing height map..." << std::endl;
  */
  
//  exit(-1);
    bot_lcmgl_switch_buffer(lcmgl_);  


}

void heightmap_lcmgl::pose_handler(const bot_core_pose_t *msg){
  bot_pose_last = bot_core_pose_t_copy (msg);
}


void heightmap_lcmgl::mapcreate_handler(const drc_map_params_t *msg){
  //bot_pose_last = bot_core_pose_t_copy (msg);
  
  cout << "got this create: " 
      << msg->transform_to_local.translation.x << ", "
      << msg->transform_to_local.translation.y << ", "
      << msg->transform_to_local.translation.z << "\n";
      hack_offset_x_ =msg->transform_to_local.translation.x;
      hack_offset_y_ =msg->transform_to_local.translation.y;
      hack_offset_z_ =msg->transform_to_local.translation.z;
}



int heightmap_lcmgl::initialize(int argc, char **argv){

  drc_heightmap_t_subscribe(subscribe_lcm_, "HEIGHT_MAP",
      heightmap_lcmgl::heightmap_handler_aux, this);

  //drc_map_params_t_subscribe(subscribe_lcm_, "MAP_CREATE",
  //    heightmap_lcmgl::mapcreate_handler_aux, this);

  
  bot_core_image_t_subscribe(subscribe_lcm_, "KINECT_RGB",
      heightmap_lcmgl::image_handler_aux, this);
  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE",
      heightmap_lcmgl::pose_handler_aux, this);

  cout << "did sub\n";
  return 1;
}


static void usage(const char *progname){
  fprintf (stderr, "usage: %s [options]\n"
      "\n"
      "  -c, --config PATH      Location of config file\n"
      , g_path_get_basename(progname));
}

int main(int argc, char ** argv) {
  string role = "robot";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(role, "r", "role","Role - robot or base");
  opt.parse();
  std::cout << "role: " << role << "\n";  
  
  lcm_t* publish_lcm;
  lcm_t* subscribe_lcm;

  if(role.compare("robot") == 0){
     subscribe_lcm= lcm_create(NULL);
  }else if(role.compare("base") == 0){
     string lcm_url = "udpm://239.255.12.68:1268?ttl=1";
     subscribe_lcm= lcm_create(lcm_url.c_str());// bot_lcm_get_global(lcm_url.c_str());
  }else{
    std::cout << "DRC Viewer role not understood, choose: robot or base\n";
    return 1;
  }  
  
  publish_lcm = subscribe_lcm;

  heightmap_lcmgl app(publish_lcm,subscribe_lcm);
  int status =  app.initialize(argc, argv);
  if (!status) { return status; }

  while(1)
    lcm_handle(subscribe_lcm);

  lcm_destroy(subscribe_lcm);
  return 0;
}
