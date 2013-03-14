// convert lidar and rgb into depth image+rgb
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

/// MAPS:
#include <pcl/range_image/range_image.h>
#include <maps/SensorDataReceiver.hpp>
#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/Collector.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/Utils.hpp>

#include <lcmtypes/multisense.hpp>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <drc_utils/Clock.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

using namespace std;
using namespace maps;

int mode =0;
int vis_offset =0;

class State {
public:
  boost::shared_ptr<lcm::LCM> mLcm;
  BotWrapper::Ptr mBotWrapper;
  boost::shared_ptr<Collector> mCollector;
  int mActiveMapId;
  bot_lcmgl_t* mLcmGl;
  
  State( boost::shared_ptr<lcm::LCM> &mLcm ): mLcm(mLcm) {
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
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, State* iState);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    std::string image_channel_;

    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   

    // Camera Params:
    int width_;
    int height_;
    double fx_, fy_, cx_, cy_;
    
    int counter_;

    pointcloud_vis* pc_vis_;
    image_io_utils*  imgutils_;
    
    bot_core::image_t disparity_;
    uint16_t *disparity_data_;
    
    bot_core::image_t img_;  
    bot_core::image_t last_img_;    
       
    // Plane Detection:
    int64_t last_sweep_time_;
    bool getSweep();
    // Point Cloud of most recent sweep:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;    
    // pose of a point on the plane with pitch and yaw but roll =0
    Eigen::Isometry3d plane_pose_ ;
    // has the above value been set?
    bool plane_pose_set_;
  protected:
    State* mState;
    
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, State* iState): lcm_(lcm_), image_channel_(image_channel_), 
    mState(iState){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  vis_offset = mode*100;
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451000,"Tracker | NPose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451001,"Tracker | Particles"           ,1,1, 4451000,1, { 0.0, 1.0, 0.0} ));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451002,"Tracker | NPose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451003,"Tracker | Plane"           ,3,1, 4451002,1, { 1.0, 0.0, 0.0} ));
  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451004,"Tracker | Plane X"           ,1,1, 4451002,1, { 0.0, 0.0, 1.0} ));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451005,"Tracker | Transform",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451006 +vis_offset,"Tracker | Poses",5,1) );
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(91004,"Pose - Camera",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(91005,"Cloud back from RI (at camera)"           ,1,1, 91004,0, { 1.0, 0.0, 1.0} ));

  
  std::string key_prefix_str = "cameras."+ image_channel_ +".intrinsic_cal";
  width_ = bot_param_get_int_or_fail(botparam_, (key_prefix_str+".width").c_str());
  height_ = bot_param_get_int_or_fail(botparam_,(key_prefix_str+".height").c_str());
  fx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fx").c_str());
  fy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fy").c_str());
  cx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cx").c_str());
  cy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cy").c_str());    
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_ );

  counter_=0;
  img_.utime=0; // used to indicate no message recieved yet
  last_img_.utime=0; // used to indicate no message recieved yet
  
  disparity_data_ = (uint16_t*) malloc(1024 * 544* 2);
  
  
  plane_pose_.setIdentity();
  plane_pose_set_ = false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_ = cloud_ptr;    
  
  
  
}




bool Pass::getSweep(){
  
  // get submap we created earlier
  LocalMap::Ptr localMap =
    mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);

  // find time range of desired swath (from 45 to 135 degrees)
  int64_t timeMin, timeMax;
  double ang_min = 5.0 *M_PI/180; // leading edge from the right hand side of sweep
  double ang_max = 175.0 *M_PI/180;
  // 0 and 180 fails
  
  int current_utime = drc::Clock::instance()->getCurrentTime();
  //cout << ang_min << " min | " << ang_max << " max\n";
        
  mState->mCollector->getLatestSwath(ang_min, ang_max,
                                        timeMin, timeMax); // these didnt work
  if (timeMin == last_sweep_time_){
    // cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | repeat\n";
    return false; 
  }
  last_sweep_time_ = timeMin;

  cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | process\n";
  LocalMap::SpaceTimeBounds bounds;
  bounds.mTimeMin = timeMin;
  bounds.mTimeMax = timeMax;

  // get and publish point cloud corresponding to this time range
  // (for debugging)
  maps::PointCloud::Ptr cloud =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
  bot_lcmgl_t* lcmgl = mState->mLcmGl;
  bot_lcmgl_color3f(lcmgl, 1, 0.75, 0.75);
  bot_lcmgl_point_size(lcmgl, 1);
  for (int i = 0; i < cloud_->size(); ++i) {
    maps::PointCloud::PointType point = (*cloud_)[i];
    bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
    bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
    bot_lcmgl_end(lcmgl);
  }
  bot_lcmgl_switch_buffer(lcmgl);  

  

  // 3. Create a depth image object:
  Eigen::Isometry3d ref_pose;
  botframes_cpp_->get_trans_with_utime( botframes_ ,  "CAMERA", "local", current_utime, ref_pose);   // ...? not sure what to use
  // set up sample camera projection parameters
  Eigen::Matrix3f calib = Eigen::Matrix3f::Identity();
  int width=1024;
  int height=544;
  double fx = 610.778; //focal length in pixels  will assume fx = fy for now
  if (1==0){ // default example
    width = 200; height = 200;  fx = 50;
  }else if (1==1){ // correct
    width =1024; height =544; fx = 610.1778;
  }else{
    width =256;  
    height =136; 
    fx = 152.54445;
  }
  double fy =fx;
  double cx = width/2.0; // do we need an etra 0.5?
  double cy = height/2.0;
  calib(0,0) =calib(1,1) = fx ;  // focal length of 50 pixels
  calib(0,2) =cx;                  // cop at center of image
  calib(1,2) =cy;

    Eigen::Projective3f projector;
  Utils::composeViewMatrix(projector, calib, isometryDoubleToFloat(ref_pose), false);
  DepthImageView::Ptr depthImageView = localMap->getAsDepthImage(width, height, projector, bounds);  
  
  
  // 6. Reproject the depths into xyz and publish
  float* depths = depthImageView->getRangeImage()->getRangesArray();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (int v = 0; v < height; v=v+4) { // rows t2b //height
    for (int u = 0; u < width; u=u+4) { // cols l2r
      pcl::PointXYZRGB pt;
      int pixel = v*width +u;
      int i_pixel = (v) *(width) +(u);
      pt.z = 1/ depths[pixel]; /// inversion currently required due to Matt's interperation of depth as 1/depth ;)
      pt.x = ( pt.z * (u  - cx))/fx ;
      pt.y = ( pt.z * (v  - cy))/fy ;
      pt.r = (float) img_.data[i_pixel*3];
      pt.g = (float) img_.data[i_pixel*3+1];
      pt.b = (float) img_.data[i_pixel*3+2];
      cloud4->points.push_back(pt);
    }
  }
  Isometry3dTime ref_poseT = Isometry3dTime(current_utime, ref_pose);
  pc_vis_->pose_to_lcm_from_list(91004, ref_poseT);  
  pc_vis_->ptcld_to_lcm_from_list(91005, *cloud4, current_utime, current_utime);  
  
  

  int n_bytes=2; // 2 bytes per value // different from before in driver
  int isize = n_bytes*1024*544;
  disparity_.utime =img_.utime;
  disparity_.width = 1024;
  disparity_.height = 544;
  disparity_.pixelformat =bot_core::image_t::PIXEL_FORMAT_FLOAT_GRAY32; //PIXEL_FORMAT_GRAY;
  disparity_.nmetadata =0;
  disparity_.row_stride=n_bytes*1024;
  disparity_.size =isize;
  disparity_.data.resize(isize);
  for (size_t i=0; i < width*height; i++){
    // convert to MM - the same as kinect mm openni format
    disparity_data_[i] = (uint16_t) 1000* 1/(depths[i]); // need 1/depth for now until Matt fixes this
  }
  memcpy(&disparity_.data[0], disparity_data_, isize);
  
  multisense::images_t images;
  images.utime = img_.utime;
  images.n_images =2;
  images.image_types.push_back( 0 ); // multisense::images_t::LEFT ); for some reason enums won't work
  images.image_types.push_back( 4 ); // multisense::images_t::DEPTH_MM );
  images.images.push_back( img_ );
  images.images.push_back(disparity_);
  lcm_->publish("MULTISENSE_LD", &images);        

  return true;
}



void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  
  if (width_ != msg->width){
    cout << "incoming width " << msg->width << " doesn't match assumed width " << width_ << "\n";
    cout << "returning cowardly\n";
    return;
  }
  img_= *msg;  
  
  if (getSweep()){
  }
  
 
  last_img_ = img_;
}


int main(int argc, char ** argv) {
  string channel = "CAMERALEFT";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel","channel");
  opt.add(mode, "m", "mode","Mode");
  opt.parse();
  std::cout << "channel: " << channel << "\n";    

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
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
  mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*10;
  mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*10;
  mapSpec.mResolution = 0.01;
  state.mActiveMapId = state.mCollector->getMapManager()->createMap(mapSpec);
  // start running wrapper
  std::string laserChannel("SCAN");
  state.mCollector->getDataReceiver()->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");
  state.mCollector->start();  
  
  
  Pass app(lcm, channel, &state);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}