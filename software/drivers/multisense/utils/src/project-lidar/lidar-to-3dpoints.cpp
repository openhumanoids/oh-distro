// minimal renderer of lidar data in lcmgl
// fully working as of august 2013

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/multisense.hpp"

using namespace std;
using namespace Eigen;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    void cloudHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::pointcloud_t* msg);   
    void camToScanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);   
    
    bot_lcmgl_t* lcmgl_;
    int printf_counter_; // used for terminal feedback
    
    std::vector<Eigen::Vector3d> cloud_incoming_; // 3D points that were precomputed
    
    std::vector<Eigen::Vector3d> cloud_transformed_; // incoming lidar converted to 3D points
    Eigen::Isometry3d camera_to_scan_; // transform used to do conversion
    
    void drawLidar(int64_t last_utime);
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-to-3dpoints");
  
  lcm_->subscribe( "SCAN" ,&Pass::lidarHandler,this);
  lcm_->subscribe( "SCAN_CLOUD" ,&Pass::cloudHandler,this);
  lcm_->subscribe( "CAMERA_TO_SCAN" ,&Pass::camToScanHandler,this);
  printf_counter_ =0;
}

void Pass::cloudHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::pointcloud_t* msg){
  cloud_incoming_.clear();
  for (size_t i=0; i < msg->num_points; ++i) {
    Eigen::Vector3d pt( msg->x[i], msg->y[i], msg->z[i]);
    cloud_incoming_.push_back(pt);
  }
  
  drawLidar(msg->utime);
}

void Pass::camToScanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  camera_to_scan_.setIdentity();
  camera_to_scan_.translation()  << msg->trans[0], msg->trans[1], msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                               msg->quat[2], msg->quat[3]);
  camera_to_scan_.rotate(quat);    
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

void bot_lcmgl_draw_axes(bot_lcmgl_t * lcmgl, const Eigen::Affine3d & trans, double scale)
{
  lcmglPushMatrix();
  lcmglMultMatrixd(trans.data());
  lcmglScalef(scale, scale, scale);
  bot_lcmgl_draw_axes(lcmgl);
  lcmglPopMatrix();
}

void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
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
  Eigen::Affine3d Aff = Eigen::Affine3d(camera_to_scan_ );
  for (size_t i=0; i < scan_laser.size(); ++i) {
    Eigen::Vector3d pt = scan_laser[i];
    pt = Aff * pt;
    cloud_transformed_.push_back(pt);
  }
  /// cloud_transformed_ now contains the lidar returns as 3d points in the camera frame
}
  
void Pass::drawLidar(int64_t last_utime){
  // transform and point cloud are both in camera optical frame (z forward, x right)
  // we need to convert to (x forward, z up) here:
  
  // a. Visualize the axes of interest:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_rotated(lcmgl_, -90, 1, 0, 0);  
  bot_lcmgl_rotated(lcmgl_, 90, 0, 1, 0); 
  bot_lcmgl_draw_axes(lcmgl_, camera_to_scan_, 4);
  
  // b. Plot scan from camera frame:
  bot_lcmgl_point_size(lcmgl_, 4.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue
  for (size_t i=0; i < cloud_transformed_.size(); ++i) {
    Eigen::Vector3d pt = cloud_transformed_[i];
    bot_lcmgl_vertex3d(lcmgl_, pt(0), pt(1), pt(2)); // 100 points in line
  }
  bot_lcmgl_end(lcmgl_);

  // c. Render incoming point cloud from camera frame
  bot_lcmgl_point_size(lcmgl_, 2.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 1, 0); // Blue
  for (size_t i=0; i < cloud_incoming_.size(); ++i) {
    Eigen::Vector3d pt = cloud_incoming_[i];
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
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Ready to visualize lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
