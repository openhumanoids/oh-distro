// renderer of lidar data in collections renderer
// uses pcl and can be used to dump cloud to file as PCD
// fully working as of august 2013

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>

#include <bot_lcmgl_client/lcmgl.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/multisense.hpp"
#include "lcmtypes/visualization.hpp"

using namespace std;
using namespace Eigen;
using namespace boost::assign; // bring 'operator+()' into scope

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
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_incoming_; // 3D points that were precomputed
    
    Eigen::Isometry3d local_to_camera_ ; 
    Eigen::Isometry3d camera_to_scan_; // transform used to do conversion
  
    pointcloud_vis* pc_vis_ ;
    bot_lcmgl_t* lcmgl_;
    int printf_counter_; // used for terminal feedback
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-to-3dpoints");
  
  lcm_->subscribe( "SCAN" ,&Pass::lidarHandler,this);
  lcm_->subscribe( "SCAN_CLOUD" ,&Pass::cloudHandler,this);
  lcm_->subscribe( "CAMERA_TO_SCAN" ,&Pass::camToScanHandler,this);
  printf_counter_ =0;
  
  bool reset =0;
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose - Laser",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Cloud - Laser"         ,1,reset, 60000,1, {0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(2000,"Pose - Null",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2001,"Cloud - Null"           ,1,reset, 2000,1, { 1.0, 1.0, 0.0} ));  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Pose - Camera",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Cloud - Camera [Interpolated]"           ,1,reset, 3000,1, { 0.0, 1.0, 1.0} ));  
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_incoming_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_incoming_ = cloud_incoming_ptr;
  
  // Tranform from Local to Camera:
  Matrix3d m;
  m = AngleAxisd (-M_PI/2, Vector3d::UnitZ ())
                  * AngleAxisd (0, Vector3d::UnitY ())
                  * AngleAxisd ( -M_PI/2, Vector3d::UnitX ());  
  local_to_camera_ =  Eigen::Isometry3d::Identity();
  local_to_camera_ *= m;  
  local_to_camera_.translation()  << 0,0,0;
}

void Pass::cloudHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::pointcloud_t* msg){
  
  cloud_incoming_->width   = msg->num_points;
  cloud_incoming_->height   = 1;
  cloud_incoming_->points.resize (msg->num_points);
  for (size_t i=0; i < msg->num_points; ++i) {
    cloud_incoming_->points[i].x = msg->x[i];
    cloud_incoming_->points[i].y = msg->y[i];
    cloud_incoming_->points[i].z = msg->z[i];
  }
  
  // Plot interpolated scan from local:
  int64_t pose_id=msg->utime;
  Isometry3dTime local_to_camera_T = Isometry3dTime(pose_id, local_to_camera_ );
  pc_vis_->pose_to_lcm_from_list(3000, local_to_camera_T);
  pc_vis_->ptcld_to_lcm_from_list(3001, *cloud_incoming_, pose_id, pose_id);  
}

void Pass::camToScanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  Eigen::Isometry3d camera_to_scan_in;
  camera_to_scan_in.setIdentity();
  camera_to_scan_in.translation()  << msg->trans[0], msg->trans[1], msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                               msg->quat[2], msg->quat[3]);
  camera_to_scan_in.rotate(quat);    
  
  camera_to_scan_ = local_to_camera_ * camera_to_scan_in;
}

// From pointcloud_lcm
void convertLidar(std::vector< float > ranges, int numPoints, double thetaStart,
        double thetaStep,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        double minRange = 0., double maxRange = 1e10,
        double validRangeStart = -1000, double validRangeEnd = 1000)
{
  int count = 0;
  double theta = thetaStart;
  cloud->width   = numPoints;
  cloud->height   = 1;
  cloud->points.resize (numPoints);
  for (int i = 0; i < numPoints; i++) {
    if (ranges[i] > minRange && ranges[i] < maxRange && theta > validRangeStart
            && theta < validRangeEnd) { 
        //hokuyo driver seems to report maxRanges as .001 :-/
        //project to body centered coordinates
        cloud->points[count].x = ranges[i] * cos(theta);
        cloud->points[count].y = ranges[i] * sin(theta);
        count++;
    }
    theta += thetaStep;
  }
  // Resize outgoing cloud
  cloud->width   = count;
  cloud->points.resize (count);
}


void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  int64_t pose_id=msg->utime;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_laser (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_camera (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  
  // 1. Convert scan into simple XY point cloud:  
  double minRange =0.0; // consider everything - don't remove any points
  double maxRange = 100.0;
  double validBeamAngles[] ={-10,10}; 
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, scan_laser, minRange, maxRange,
      validBeamAngles[0], validBeamAngles[1]);  
  
  // 2. Project the scan into local frame:
  Eigen::Isometry3f camera_to_scan_f= camera_to_scan_.cast<float>();
  pcl::transformPointCloud (*scan_laser, *scan_camera,
      camera_to_scan_f.translation(), Eigen::Quaternionf(camera_to_scan_f.rotation())  );  
  
  ///
  /// scan_camera now contains the lidar returns as 3d points in the camera frame
  ///
  
  // Plot original scan from lidar frame:
  Isometry3dTime camera_to_scan_T = Isometry3dTime(pose_id, camera_to_scan_);
  pc_vis_->pose_to_lcm_from_list(60000, camera_to_scan_T);
  pc_vis_->ptcld_to_lcm_from_list(60001, *scan_laser, pose_id, pose_id);  

  // Plot scan from camera frame:
  Isometry3dTime null_T = Isometry3dTime(pose_id, Eigen::Isometry3d::Identity() );
  pc_vis_->pose_to_lcm_from_list(2000, null_T);
  pc_vis_->ptcld_to_lcm_from_list(2001, *scan_camera, pose_id, pose_id);  
  
  if (printf_counter_%200 ==0){
    cout << "Filtering: " <<  " "  << msg->utime << "\n";
      
    vs::reset_collections_t reset;
    lcm_->publish("RESET_COLLECTIONS", &reset);    
  }
  printf_counter_++;
}


int main( int argc, char** argv ){
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
