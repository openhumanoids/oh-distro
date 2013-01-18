// Lidar Pass trough filter
// TODO: get the urdf model from LCM:

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
#include <bot_lcmgl_client/lcmgl.h>


#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <pointcloud_tools/pointcloud_lcm.hpp> // unpack lidar to xyz
#include <lcmtypes/drc_lcmtypes.hpp>
#include "lcmtypes/bot_core.hpp"
#include <ConciseArgs>

using namespace std;
using namespace Eigen;
using namespace boost::assign; // bring 'operator+()' into scope


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;

    void affordanceCollectionHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_collection_t* msg);   
    void headPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);   
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;
    bot_lcmgl_t* lcmgl_;
    
    Isometry3dTime current_poseT_;
    
    pointcloud_vis* pc_vis_;
    pointcloud_lcm* pc_lcm_;
    int vis_counter_; // used for visualization

};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_):
    lcm_(lcm_), verbose_(verbose_),
    current_poseT_(0, Eigen::Isometry3d::Identity()){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  lcmgl_= bot_lcmgl_init(lcm_->getUnderlyingLCM(), "lidar-pt");
  lcm_->subscribe( "AFFORDANCE_COLLECTION" ,&Pass::affordanceCollectionHandler,this);
  lcm_->subscribe("POSE_HEAD",&Pass::headPoseHandler,this);
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Null - World",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Affordance - World"           ,1,1, 1000,1, { 0.0, 1.0, 0.0} ));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1002,"Head - World",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1003,"Affordance - Head"           ,1,1, 1002,1, { 0.0, 1.0, 0.0} ));
  
  vis_counter_ =0;  
  cout << "Finished setting up\n";
}



// Get look angles to relative point
// Yaw: positive to the left
// Pitch: positive up 
void getLookAngles(Eigen::Vector3d rel_point, double &yaw, double &pitch){
  yaw  = atan2( rel_point(1) , rel_point(0) );
  pitch= atan2( rel_point(2) , rel_point(0) );
}


void Pass::affordanceCollectionHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_collection_t* msg){
  cout << "Affordances: "  << " "  << msg->naffs << "\n";
  
  int id=1;
  
  int64_t utime = current_poseT_.utime;

  /// Visualise incoming point in world frame:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointXYZRGB pt_in;
  pt_in.x = msg->affs[id].params[0];
  pt_in.y = msg->affs[id].params[1];
  pt_in.z = msg->affs[id].params[2];
  scan_cloud->points.push_back(pt_in);
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(utime, null_pose);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT);  
  pc_vis_->ptcld_to_lcm_from_list(1001, *scan_cloud, utime, utime);

  
  /// Find the position of the affordance relative to the sensor head:
  /// The only interesting thing:
  Vector3d point(msg->affs[id].params[0], msg->affs[id].params[1], msg->affs[id].params[2]) ;
  Vector3d rel_point = current_poseT_.pose.inverse() *point;
  double yaw,pitch;
  getLookAngles(rel_point,yaw,pitch);
  std::cout << "Affordance[0] " << msg->affs[id].name << " is: \n";
  std::cout << "pitch: " << pitch << " and yaw " << yaw << "\n";

  /// Visualize outgoing point in pose frame:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointXYZRGB pt_out;
  pt_out.x = rel_point(0);
  pt_out.y = rel_point(1);
  pt_out.z = rel_point(2);
  cloud_local->points.push_back(pt_out); 
  pc_vis_->pose_to_lcm_from_list(1002, current_poseT_);
  pc_vis_->ptcld_to_lcm_from_list(1003, *cloud_local, utime, utime);
}

void Pass::headPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  current_poseT_.pose.setIdentity();
  current_poseT_.pose.translation() << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond m;
  m  = Eigen::Quaterniond(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);
  current_poseT_.pose.rotate(m);
  current_poseT_.utime = msg->utime;
  
  
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.parse();
  cout << verbose << " is verbose\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose);
  cout << "Ready to find " << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
