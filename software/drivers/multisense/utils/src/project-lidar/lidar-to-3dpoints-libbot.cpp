// minimal module to convert lidar into world frame xyz points
// mfallon august 2013

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>

#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/multisense.hpp"

#include <ConciseArgs>

using namespace std;
using namespace Eigen;

struct CommandLineConfig
{
    std::string lidar_channel;
};

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig& cl_cfg_;
    
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    
    int get_trans_with_utime(std::string from_frame, std::string to_frame, 
                                 int64_t utime, Eigen::Isometry3d & mat);
    
    bot_lcmgl_t* lcmgl_;
    BotParam* botparam_;
    BotFrames* botframes_;
    
    int printf_counter_; // used for terminal feedback
    Eigen::Isometry3d scan_to_local_; // transform used to do conversion
    std::vector<Eigen::Vector3d> cloud_transformed_; // incoming lidar converted to 3D points
    void drawLidar(int64_t last_utime);    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-to-3dpoints");
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  lcm_->subscribe( cl_cfg_.lidar_channel  ,&Pass::lidarHandler,this);
  printf_counter_ =0;  
}


// from pointcloud_lcm
void convertLidar(std::vector< float > ranges, int numPoints, double thetaStart,
        double thetaStep,
        std::vector<Eigen::Vector3d> &cloud,
        double minRange = 0., double maxRange = 1e10,
        double validRangeStart = -1000, double validRangeEnd = 1000)
{
  double theta = thetaStart;
  for (int i = 0; i < numPoints; i++) {
    if (ranges[i] > minRange && ranges[i] < maxRange && theta > validRangeStart
            && theta < validRangeEnd) { 
      Eigen::Vector3d pt;
      pt(0) = ranges[i] * cos(theta);
      pt(1) = ranges[i] * sin(theta);
      pt(2) = 0;
      cloud.push_back(pt);
    }
    theta += thetaStep;
  }
}


int Pass::get_trans_with_utime(std::string from_frame, std::string to_frame, 
                                 int64_t utime, Eigen::Isometry3d & mat){
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( botframes_, from_frame.c_str(),  to_frame.c_str(), utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }  
  return status;
}


void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  get_trans_with_utime(  "SCAN", "local"  , msg->utime, scan_to_local_);
    
  // 1. Convert scan into simple XY point cloud:  
  std::vector<Eigen::Vector3d> scan_laser;
  double minRange =0.0; // consider everything - don't remove any points
  double maxRange = 100.0;
  double validBeamAngles[] ={-10,10}; 
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, scan_laser, minRange, maxRange,
      validBeamAngles[0], validBeamAngles[1]);    

  // 2. Project the scan into camera frame:
  cloud_transformed_.clear();
  Eigen::Affine3d Aff = Eigen::Affine3d(scan_to_local_ );
  for (size_t i=0; i < scan_laser.size(); ++i) {
    Eigen::Vector3d pt = scan_laser[i];
    pt = Aff * pt;
    cloud_transformed_.push_back(pt);
  }
  /// cloud_transformed_ now contains the lidar returns as 3d points in the local frame
  
  drawLidar(msg->utime);
}

void bot_lcmgl_draw_axes(bot_lcmgl_t * lcmgl, const Eigen::Affine3d & trans, double scale)
{
  lcmglPushMatrix();
  lcmglMultMatrixd(trans.data());
  lcmglScalef(scale, scale, scale);
  bot_lcmgl_draw_axes(lcmgl);
  lcmglPopMatrix();
}

void Pass::drawLidar(int64_t last_utime){
  // a. Visualize the axes of interest:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_draw_axes(lcmgl_, scan_to_local_, 4);
  
  // b. Plot scan from camera frame:
  bot_lcmgl_point_size(lcmgl_, 4.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue
  for (size_t i=0; i < cloud_transformed_.size(); ++i) {
    Eigen::Vector3d pt = cloud_transformed_[i];
    bot_lcmgl_vertex3d(lcmgl_, pt(0), pt(1), pt(2)); // 100 points in line
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);  
  bot_lcmgl_switch_buffer(lcmgl_);  

  if (printf_counter_%80 ==0){
    cout << "Scan @ "  << last_utime << "\n";
  }
  printf_counter_++;
}

int main( int argc, char** argv ){
  CommandLineConfig cl_cfg;
  cl_cfg.lidar_channel = "SCAN";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.lidar_channel, "l", "lidar_channel","lidar_channel");
  opt.parse();
  std::cout << "lidar_channel: " << cl_cfg.lidar_channel << "\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, cl_cfg);
  cout << "Ready to visualize lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
