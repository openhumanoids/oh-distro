// Grab a single sweep of lidar and do various tasks:
// - convert to a point cloud
// - convert to depth image (with rgb)
// - apply the affordance mask to the depth image
// - republish for Dense Depth Affordance.

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
#include <maps/DepthImage.hpp>

#include <lcmtypes/multisense.hpp>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <drc_utils/Clock.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

using namespace std;
using namespace maps;

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
    void maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   

    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   

    // Camera Params:
    int width_;
    int height_;
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3f camera_calib_;
    
    int counter_;

    // Get a sweep of lidar
    // True if we have a new sweep, different from before
    bool getSweep(LocalMap::SpaceTimeBounds bounds, Eigen::Vector3f bounds_center = Eigen::Vector3f(0,0,0), 
		 Eigen::Vector3f bounds_size = Eigen::Vector3f(1e10, 1e10, 1e10) );

    void getSweepCloud(LocalMap::SpaceTimeBounds bounds);
    void sendSweepCloud();
    void getSweepDepthImage(LocalMap::SpaceTimeBounds bounds);
    void sendSweepDepthImage();

    pointcloud_vis* pc_vis_;
    image_io_utils*  imgutils_;
    
    bot_core::image_t disparity_;
    uint16_t *disparity_data_;
    
    bot_core::image_t img_;  
       
    // Plane Detection:
    int64_t last_sweep_time_;
    // Point Cloud of most recent sweep:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;  
    // Depth buffer:
    std::vector<float> depth_buf_;

    Eigen::Isometry3d camera_pose_;
    int64_t current_utime_;

    bot_core::image_t last_mask_;    
    bool mask_init_;

  protected:
    State* mState;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, State* iState): lcm_(lcm_), image_channel_(image_channel_), 
    mState(iState){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);
  string mask_channel="CAMERALEFT_MASKZIPPED";
  lcm_->subscribe( mask_channel ,&Pass::maskHandler,this);


  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(91000,"Null Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(91001,"Raw Sweep Cloud"           ,1,1, 91000,1, { 0.0, 1.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(91004,"Pose - Camera",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(91005,"Range Image as Cloud"           ,1,1, 91004,0, { 1.0, 0.0, 1.0} ));

  
  std::string key_prefix_str = "cameras."+ image_channel_ +".intrinsic_cal";
  width_ = bot_param_get_int_or_fail(botparam_, (key_prefix_str+".width").c_str());
  height_ = bot_param_get_int_or_fail(botparam_,(key_prefix_str+".height").c_str());
  fx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fx").c_str());
  fy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fy").c_str());
  cx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cx").c_str());
  cy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cy").c_str());    

  // set up camera projection parameters - for depth image from camera
  camera_calib_ = Eigen::Matrix3f::Identity();
  camera_calib_(0,0) = camera_calib_(1,1) = fx_ ;  // focal length
  camera_calib_(0,2) =cx_;                  // cop at center of image
  camera_calib_(1,2) =cy_;

  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_ );

  counter_=0;
  
  disparity_data_ = (uint16_t*) malloc(width_ * height_* 2);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_ = cloud_ptr;    

  mask_init_=false;
}




bool Pass::getSweep(LocalMap::SpaceTimeBounds bounds, Eigen::Vector3f bounds_center, Eigen::Vector3f bounds_size){
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
//  bounds.mPlanes = Utils::planesFromBox(   Eigen::Vector3f(-3,-3,-3),
//                                          Eigen::Vector3f(3,3,3));
  bounds.mPlanes = Utils::planesFromBox( bounds_center - bounds_size, bounds_center + bounds_size );

  return true;
}

// Extract Sweep as a Cloud:
void Pass::getSweepCloud(LocalMap::SpaceTimeBounds bounds){
  LocalMap::Ptr localMap = mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);
  // get point cloud corresponding to this time range:
  cloud_ =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
}
  
// Extract Sweep as a Depth Image:
void Pass::getSweepDepthImage(LocalMap::SpaceTimeBounds bounds){
  LocalMap::Ptr localMap = mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);

  Eigen::Projective3f projector;
  Utils::composeViewMatrix(projector, camera_calib_, isometryDoubleToFloat(camera_pose_), false);
  DepthImageView::Ptr depthImageView = localMap->getAsDepthImage(width_, height_, projector, bounds);  
  depth_buf_ = depthImageView->getDepthImage()->getData(DepthImage::TypeDisparity);
}  
/////////////////////////////////////////////////////////////////////////////


// Publish Various Representations of the Depth Image:
void Pass::sendSweepDepthImage(){
  {
  // a. Write raw depths to file
  if (1==0){
    std::ofstream ofs("/tmp/sweep_depths.txt");
    for (int i = 0; i < height_; ++i) {
      for (int j = 0; j < width_; ++j) {
        ofs << depth_buf_[i*width_ + j] << " ";
      }
      ofs << std::endl;
    }
    ofs.close();
  }

  cout << "a\n";

  // b. Reproject the depth image into xyz, colourize, apply a mask and publish
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  int decimate=4;
  uint8_t* mask_buf;
  if (mask_init_){
    mask_buf = imgutils_->unzipImage( &last_mask_ );
  }

  cout << "b\n";
  
  for (int v = 0; v < height_; v=v+decimate) { // rows t2b //height
    for (int u = 0; u < width_; u=u+decimate) { // cols l2r
      pcl::PointXYZRGB pt;
      int pixel = v*width_ +u;
      pt.z = 1/ depth_buf_[pixel]; /// inversion currently required due to Matt's interperation of depth as 1/depth ;)
      pt.x = ( pt.z * (u  - cx_))/fx_ ;
      pt.y = ( pt.z * (v  - cy_))/fy_ ;
      if (img_.pixelformat == bot_core::image_t::PIXEL_FORMAT_RGB){
        pt.r = (float) img_.data[pixel*3];
        pt.g = (float) img_.data[pixel*3+1];
        pt.b = (float) img_.data[pixel*3+2];
      }else if (img_.pixelformat == bot_core::image_t::PIXEL_FORMAT_GRAY){
        pt.r = (float) img_.data[pixel];
        pt.g = (float) img_.data[pixel];
        pt.b = (float) img_.data[pixel];
      }

      if (mask_init_){ // if we have a mask color the points by it
        if (mask_buf[pixel] > 0){ // if the mask is not 0 (black), apply it as red
          pt.r = 255;
          pt.g = 0;
          pt.b = 0;
        }
      }

      cloud4->points.push_back(pt);
    }
  }
  cout << "c\n";
  
  Isometry3dTime camera_pose_T = Isometry3dTime(current_utime_, camera_pose_);
  pc_vis_->pose_to_lcm_from_list(91004, camera_pose_T);  
  pc_vis_->ptcld_to_lcm_from_list(91005, *cloud4, current_utime_, current_utime_);  
}

  // c. publish in Depth Image mode:
  int n_bytes=2; // 2 bytes per value // different from before in driver
  int isize = n_bytes*width_*height_;
  disparity_.utime =img_.utime;
  disparity_.width = width_;
  disparity_.height = height_;
  disparity_.pixelformat =bot_core::image_t::PIXEL_FORMAT_FLOAT_GRAY32; //PIXEL_FORMAT_GRAY;
  disparity_.nmetadata =0;
  disparity_.row_stride=n_bytes*width_;
  disparity_.size =isize;
  disparity_.data.resize(isize);
  for (size_t i=0; i < width_*height_; i++){
    // convert to MM - the same as kinect mm openni format
    disparity_data_[i] = (uint16_t) 1000* 1/(depth_buf_[i]); // need 1/depth for now until Matt fixes this
  }
  memcpy(&disparity_.data[0], disparity_data_, isize);
  
  multisense::images_t images;
  images.utime = img_.utime;
  images.n_images =2;
  images.image_types.push_back( 0 ); // multisense::images_t::LEFT ); for some reason enums won't work
  images.image_types.push_back( 4 ); // multisense::images_t::DEPTH_MM );
  images.images.push_back( img_ );
  images.images.push_back(disparity_);
  lcm_->publish("LIDARSWEEP_LD", &images);
  //lcm_->publish("MULTISENSE_LD", &images);        
}


// Send the cloud
void Pass::sendSweepCloud(){
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
        cout << "finished writing PCD to:\n" << ss2.str() <<"\n";
      }
    }

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
  current_utime_ = msg->utime;

  Eigen::Isometry3d head_to_local;
  botframes_cpp_->get_trans_with_utime( botframes_ , "head", "local"  , current_utime_, head_to_local);

  // 3. Create a depth image object:
  botframes_cpp_->get_trans_with_utime( botframes_ ,  "CAMERA", "local", current_utime_, camera_pose_);   // ...? not sure what to use


  LocalMap::SpaceTimeBounds bounds;
  if (getSweep(bounds, head_to_local.cast<float>().translation() ,  Eigen::Vector3f( 3., 3., 3.)) ){ 

    // use the time and space bounds to get a new cloud
//    getSweepCloud(bounds);
    // use the time and space bounds to get a new depth image
    getSweepDepthImage(bounds);

//    sendSweepCloud();

    sendSweepDepthImage();
  }
}


void Pass::maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  last_mask_= *msg;
  if (!mask_init_){  cout << "got mask\n";  }
  mask_init_ = true;
}


int main(int argc, char ** argv) {
  string channel = "CAMERALEFT";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel","channel");
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
  // enabling these fixes the box in world frame
  //mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*10;
  //mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*10;
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
