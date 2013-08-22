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

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/multisense.hpp"
#include "lcmtypes/visualization.hpp"

#include <camera_params/camera_params.hpp>     // Camera Parameters
#include <ConciseArgs>

using namespace std;
using namespace Eigen;
using namespace boost::assign; // bring 'operator+()' into scope

struct CommandLineConfig
{
    int batch_size;
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
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    
    pointcloud_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    CameraParams camera_params_;   
    
    bot_core::image_t img_;  
    int printf_counter_; // used for terminal feedback
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  lcm_->subscribe( cl_cfg_.lidar_channel  ,&Pass::lidarHandler,this);
  lcm_->subscribe( "CAMERA" ,&Pass::multisenseHandler,this);

  camera_params_ = CameraParams();
  camera_params_.setParams(botparam_, "cameras.CAMERA_LEFT");
  
  img_.utime =0;
  printf_counter_ =0;
  
  bool reset =0;
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose - Laser",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Cloud - Laser"         ,1,reset, 60000,1, {0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(2000,"Pose - Camera",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2001,"Cloud - Camera"           ,1,reset, 2000,1, { 1.0, 1.0, 0.0} ));  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2002,"Cloud - Camera Color"           ,1,reset, 2000,0, { 1.0, 1.0, 0.0} ));  
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

void Pass::multisenseHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  multisense::images_t* msg){
  cout << "got image\n";
  img_= msg->images[0];  
}

void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  if (img_.utime == 0){
    cout << "no image yet\n";
    return;
  }
  
  int64_t pose_id= printf_counter_;//msg->utime;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_laser (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_camera (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  // 1. Convert scan into simple XY point cloud:  
  double minRange =0.0; // consider everything - don't remove any points
  double maxRange = 30.0;
  double validBeamAngles[] ={-10,10}; 
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, scan_laser, minRange, maxRange,
      validBeamAngles[0], validBeamAngles[1]);  
  
  Eigen::Isometry3d scan_to_local;
  botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN", "local"  , msg->utime, scan_to_local);
  Eigen::Isometry3d camera_to_local;
  botframes_cpp_->get_trans_with_utime( botframes_ , "CAMERA_LEFT", "local"  , msg->utime, camera_to_local);
  Eigen::Isometry3d scan_to_camera;
  botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN", "CAMERA_LEFT"  , msg->utime, scan_to_camera);
  
  // Plot original scan from lidar frame:
  Isometry3dTime scan_to_local_T = Isometry3dTime(pose_id, scan_to_local);
  pc_vis_->pose_to_lcm_from_list(60000, scan_to_local_T);
  pc_vis_->ptcld_to_lcm_from_list(60001, *scan_laser, pose_id, pose_id);  

  // 2. Project the scan into camera frame:
  Eigen::Isometry3f scan_to_camera_f= scan_to_camera.cast<float>();
  pcl::transformPointCloud (*scan_laser, *scan_camera,
      scan_to_camera_f.translation(), Eigen::Quaternionf(scan_to_camera_f.rotation())  );  
  /// scan_camera now contains the lidar returns as 3d points in the camera frame
  
  // Plot scan from camera frame:
  /// Isometry3dTime null_T = Isometry3dTime(pose_id, Eigen::Isometry3d::Identity() ) ;//camera_to_local );
  Isometry3dTime null_T = Isometry3dTime(pose_id, camera_to_local );
  pc_vis_->pose_to_lcm_from_list(2000, null_T);
  pc_vis_->ptcld_to_lcm_from_list(2001, *scan_camera, pose_id, pose_id);  
  
  //  double fx =  588.7705688476562; // 05
  double fx = camera_params_.fx;// 557.1886596679688; // 02
  double  fy = fx;
  for (size_t i=0 ; i< scan_camera->points.size() ; i++){
    pcl::PointXYZRGB pt = scan_camera->points[i];
    
    if (pt.z > 0){
        pt.r = 0; pt.g =255; pt.b=0;
        double u = pt.x *fx/pt.z;
        double v = pt.y *fy/pt.z;
        
      if ( ( fabs(u)< camera_params_.cx ) && ( fabs(v)< camera_params_.cy ) ){
        //  std::cout << u << " " << v << "\n";   
        u =u+ camera_params_.cx;//   512;
        v=v+camera_params_.cy;// 272;
        int pixel = round(u) + round(v)* camera_params_.width;// 1024;
        pt.r = (float) img_.data[pixel*3];
        pt.g = (float) img_.data[pixel*3+1];
        pt.b = (float) img_.data[pixel*3+2];          
      }
    }else{
        pt.r = 255; pt.g =0; pt.b=0;
    }
    scan_camera->points[i] = pt;
  }
  pc_vis_->ptcld_to_lcm_from_list(2002, *scan_camera, pose_id, pose_id);  
  
  if (printf_counter_% cl_cfg_.batch_size ==0){
    cout << "Filtering: " <<  " "  << msg->utime << "\n";
    //  vs::reset_collections_t reset;
    //  lcm_->publish("RESET_COLLECTIONS", &reset);    
    printf_counter_=0;
  }
  printf_counter_++;
}


int main( int argc, char** argv ){
  CommandLineConfig cl_cfg;
  cl_cfg.batch_size =200;
  cl_cfg.lidar_channel = "SCAN";
  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.lidar_channel, "l", "lidar_channel","lidar_channel");
  opt.add( cl_cfg.batch_size, "s", "size","Batch Size");
  opt.parse();
  std::cout << "lidar_channel: " << cl_cfg.lidar_channel << "\n"; 
  std::cout << "size: " << cl_cfg.batch_size<< "\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, cl_cfg);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
