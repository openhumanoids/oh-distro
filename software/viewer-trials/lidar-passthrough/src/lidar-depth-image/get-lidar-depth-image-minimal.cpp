// Grab a single sweep of lidar and visualize in viewer
// 

#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

/// MAPS:
#include <maps/SensorDataReceiver.hpp>
#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/Collector.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/Utils.hpp>
#include <maps/DepthImage.hpp>

#include <drc_utils/Clock.hpp>

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <ConciseArgs>

using namespace std;
using namespace maps;

class State {
public:
  std::shared_ptr<lcm::LCM> mLcm;
  BotWrapper::Ptr mBotWrapper;
  std::shared_ptr<Collector> mCollector;
  int mActiveMapId;
  bot_lcmgl_t* mLcmGl;
  
  State( std::shared_ptr<lcm::LCM> &mLcm ): mLcm(mLcm) {
    mBotWrapper.reset(new BotWrapper(mLcm));
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mActiveMapId = 0;
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "test-points");
    drc::Clock::instance()->setLcm(mLcm);
  }
  
  ~State() {
    bot_lcmgl_destroy(mLcmGl);
  }
};


class Pass{
  public:
    Pass(std::shared_ptr<lcm::LCM> &lcm_, State* iState);
    
    ~Pass(){
    }    
  private:
    std::shared_ptr<lcm::LCM> lcm_;

    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
        
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::planar_lidar_t* msg);

    // Get a sweep of lidar
    // True if we have a new sweep, different from before
    bool getSweep(LocalMap::SpaceTimeBounds& bounds, Eigen::Vector3f bounds_center = Eigen::Vector3f(0,0,0), 
                 Eigen::Vector3f bounds_size = Eigen::Vector3f(1e10, 1e10, 1e10) );

    void sendSweepCloud(LocalMap::SpaceTimeBounds bounds);

    pronto_vis* pc_vis_;
    
    int64_t last_sweep_time_;
    // Point Cloud of most recent sweep:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;  

    int64_t current_utime_;

  protected:
    State* mState;
};

Pass::Pass(std::shared_ptr<lcm::LCM> &lcm_,  State* iState): lcm_(lcm_), 
    mState(iState){

  lcm_->subscribe("SCAN",&Pass::lidarHandler,this);  
      
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_ = cloud_ptr;    

  
  // Vis Config:
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(91000,"Null Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(91001,"Raw Sweep Cloud"           ,1,1, 91000,1, { 0.0, 1.0, 1.0} ));
}


bool Pass::getSweep(LocalMap::SpaceTimeBounds& bounds, Eigen::Vector3f bounds_center, Eigen::Vector3f bounds_size){
  // get submap we created earlier
  LocalMap::Ptr localMap = mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);

  // find time range of desired swath
  int64_t timeMin, timeMax;
  double ang_min = 0.0 *M_PI/180; // leading edge from the right hand side of sweep
  double ang_max = 179.99 *M_PI/180;  // 0 and 180 fails
  //cout << ang_min << " min | " << ang_max << " max\n";
        
  bool gotFirstSweep = mState->mCollector->getLatestSwath(ang_min, ang_max,
                                        timeMin, timeMax);
  if (!gotFirstSweep){ // have not properly init'ed the collector - not a full sweep yet
    // cout << "not prop init yet\n"; 
    return false;
  }

  if (timeMin == last_sweep_time_){ // Is the sweep the same as last time?
    // cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | repeat\n";
    return false; 
  }
  last_sweep_time_ = timeMin;
  cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime_ << " utime | process\n";

  bounds.mTimeMin = timeMin;
  bounds.mTimeMax = timeMax;
  // Also add constraints to that the points are around the robot:
  // axis aligned box thats 6x6x6m
  bounds.mPlanes = Utils::planesFromBox( bounds_center - bounds_size, bounds_center + bounds_size );

  return true;
}

// Extract Sweep as a Cloud:
// Send the cloud to viewer
void Pass::sendSweepCloud(LocalMap::SpaceTimeBounds bounds){

  LocalMap::Ptr localMap = mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);
  // get point cloud corresponding to this time range:
  cloud_ =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
  
  if (1==0){
    bot_lcmgl_t* lcmgl = mState->mLcmGl;
    bot_lcmgl_color3f(lcmgl, 1, 0.75, 0.75);
    bot_lcmgl_point_size(lcmgl, 2); //1
    for (int i = 0; i < cloud_->size(); ++i) {
      maps::PointCloud::PointType point = (*cloud_)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_switch_buffer(lcmgl); 
  }

  // Republish as a point cloud (same as above)
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(current_utime_, null_pose);
  pc_vis_->pose_to_lcm_from_list(91000, null_poseT);  
  pc_vis_->ptcld_to_lcm_from_list(91001, *cloud_, current_utime_, current_utime_);      
  
  // Write as a PCD file:
  if(1==1){
    if (cloud_->points.size() > 0) {
      pcl::PCDWriter writer;
      stringstream ss2;
      ss2 << "/tmp/sweep_cloud_"  << current_utime_ << ".pcd";
      writer.write (ss2.str(), *cloud_, false);
      cout << "finished writing "<< cloud_->points.size() <<" points to:\n" << ss2.str() <<"\n";
      
    }
  }
}


void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                             const  bot_core::planar_lidar_t* msg){
  current_utime_ = msg->utime;

  Eigen::Isometry3d head_to_local;
  botframes_cpp_->get_trans_with_utime( botframes_ , "head", "local"  , current_utime_, head_to_local);

  LocalMap::SpaceTimeBounds bounds;
  if (getSweep(bounds, head_to_local.cast<float>().translation() ,  Eigen::Vector3f( 1.3, 1.3, 1.3)) ){ 
    // use the time and space bounds to get a new cloud
    sendSweepCloud(bounds);
  }
}


int main(int argc, char ** argv) {
  
  string lidar_channel = "SCAN";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(lidar_channel, "l", "lidar_channel","lidar_channel");
  opt.parse();
  std::cout << "lidar_channel: " << lidar_channel << "\n"; 

  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  // create state object instance
  State state(lcm);
  // create new submap
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mResolution = 0.01;
  state.mActiveMapId = state.mCollector->getMapManager()->createMap(mapSpec);
  // start running wrapper
  std::string laserChannel( lidar_channel );
  state.mCollector->getDataReceiver()->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");
  state.mCollector->start();  
  
  Pass app(lcm, &state);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
