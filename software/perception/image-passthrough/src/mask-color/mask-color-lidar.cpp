#include <fstream>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <pcl/range_image/range_image.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>
#include <maps/SensorDataReceiver.hpp>
#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/Collector.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/Utils.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>


#include <drc_utils/Clock.hpp>


#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <lcmtypes/bot_core.hpp>


using namespace maps;
using namespace std;


class State {
public:
  boost::shared_ptr<lcm::LCM> mLcm;
  BotWrapper::Ptr mBotWrapper;
  boost::shared_ptr<Collector> mCollector;
  int mActiveMapId;
  bot_lcmgl_t* mLcmGl;
  
  std::string mask_channel_;
  void maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
  bot_core::image_t last_mask_;    
  
  image_io_utils*  imgutils_;

  uint8_t* getMask(){
      return imgutils_->unzipImage(   &last_mask_ );
  }
  
  
  

  State( boost::shared_ptr<lcm::LCM> &mLcm  ): mLcm(mLcm) {
    mBotWrapper.reset(new BotWrapper(mLcm));
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mActiveMapId = 0;
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "test-points");
    
    drc::Clock::instance()->setLcm(mLcm);
    
    mask_channel_="CAMERALEFT_MASKZIPPED";
    mLcm->subscribe( mask_channel_ ,&State::maskHandler,this);
    last_mask_.utime =0; // use this number to determine initial image
    
    
    imgutils_ = new image_io_utils( mLcm->getUnderlyingLCM(), 1024, 544);
    
  }
  
  ~State() {
    bot_lcmgl_destroy(mLcmGl);
  }
};


void State::maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  last_mask_= *msg;  
  cout << "got mask\n";  
}


class DataProducer {
private:  
  
  // Added by mfallon:
  BotParam* botparam_;
  BotFrames* botframes_;
  bot::frames* botframes_cpp_;    
  pointcloud_vis* pc_vis_;

  int64_t last_timeMin_;
  protected:
    State* mState;  
public:
  DataProducer(State* iState);
  
  void operator()();
};



DataProducer::DataProducer(State* iState) : mState(iState) {
  botparam_ = bot_param_new_from_server(mState->mLcm->getUnderlyingLCM(), 0);  
  botframes_= bot_frames_get_global(mState->mLcm->getUnderlyingLCM(), botparam_);  
  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( mState->mLcm->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(91000,"Pose - Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(91001,"Cloud - Null"           ,1,1, 91000,1, { 0.0, 1.0, 1.0} ));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(91002,"Pose - Camera [R]",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(91003,"Cloud using in RI"           ,1,1, 91000,0, { 1.0, 0.0, 1.0} ));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(91004,"Pose - Camera",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(91005,"Cloud back from RI (at camera)"           ,1,1, 91004,0, { 1.0, 0.0, 1.0} ));
  
}

void DataProducer::operator()() {
  const float kPi = 4*tan(1);
  const float degToRad = kPi/180;
  while (true) {
    // get submap we created earlier
    LocalMap::Ptr localMap =
      mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);

    // find time range of desired swath (from 45 to 135 degrees)
    int64_t timeMin, timeMax;
    double ang_min = 5 *M_PI/180; // leading edge from the right hand side of sweep
    double ang_max = 180 *M_PI/180;
    //double ang_min = 45*degToRad;
    //double ang_max= 135*degToRad;
    
    int current_utime = drc::Clock::instance()->getCurrentTime();
    //cout << ang_min << " min | " << ang_max << " max\n";
         
    mState->mCollector->getLatestSwath(ang_min, ang_max,
                                         timeMin, timeMax); // these didnt work
    cout << timeMin << " timeMin | " << timeMax << " timeMax | "
         << current_utime << " utime\n";
    if (timeMin == last_timeMin_){
      cout << "repeat\n";
      // wait for timer expiration
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      timer.expires_from_now(boost::posix_time::seconds(1));
      timer.wait();      
      continue; 
    }
    last_timeMin_ = timeMin;
    LocalMap::SpaceTimeBounds bounds;
    bounds.mTimeMin = timeMin;
    bounds.mTimeMax = timeMax;
    ///////////////////////////// Data Request Completed
    

    // 1. get and publish point cloud corresponding to this time range
    maps::PointCloud::Ptr cloud =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
    bot_lcmgl_t* lcmgl = mState->mLcmGl;
    bot_lcmgl_color3f(lcmgl, 0, 1, 0);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType point = (*cloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_switch_buffer(lcmgl);

    
    // 2. Output the cloud in world frame - same as 1:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    cloud2 =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
    Eigen::Isometry3d null_pose;
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(current_utime, null_pose);
    pc_vis_->pose_to_lcm_from_list(91000, null_poseT);  
    pc_vis_->ptcld_to_lcm_from_list(91001, *cloud2, current_utime, current_utime);      
    
    if(1==0){
      if (cloud2->points.size() > 0) {
        pcl::PCDWriter writer;
        stringstream ss2;
        ss2 << "/home/mfallon/drc/software/perception/vehicle-tracker/data/vehicle_"  << timeMax << ".pcd";
        writer.write (ss2.str(), *cloud2, false);
      }
    }
    
    
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
    }else if (1==0){ // correct
      width =1024; height =544; fx = 610.1778;
    }else{
      width =256;  
      height =136; 
      fx = 152.54445;
    }
    double fy =fx;
    double cx = width/2.0;
    double cy = height/2.0;
    calib(0,0) =calib(1,1) = fx ;  // focal length of 50 pixels
    calib(0,2) =cx;                  // cop at center of image
    calib(1,2) =cy;

      Eigen::Projective3f projector;
    Utils::composeViewMatrix(projector, calib, isometryDoubleToFloat(ref_pose), false);
    DepthImageView::Ptr depthImageView = localMap->getAsDepthImage(width, height, projector, bounds);
      
      
    // Convert Pose from CV Camera Frame to Robot Camera Frame
    Eigen::Isometry3d fixrotation_pose;
    fixrotation_pose.setIdentity();
    fixrotation_pose.translation() << 0,0,0;    
    Eigen::Quaterniond fix_r = euler_to_quat(0.0*M_PI/180.0, -90.0*M_PI/180.0 , 90.0*M_PI/180.0);
    fixrotation_pose.rotate(fix_r);    
    Eigen::Isometry3d ref_pose_ROBOT = ref_pose*fixrotation_pose; 
    

    // 4. Convert Depth Image to Point Cloud and colourize using Mask (if available)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    cloud3= depthImageView->getAsPointCloud();
    int decimate_=4;
    if( mState->last_mask_.utime!=0){
      cout <<"got mask and depth\n";     
      uint8_t* mask_buf =  mState->getMask();
      //imgutils_->sendImage(mask_buf, msg->utime, 1024, 544, 1, string("UNZIPPED")  );
      // Colorise the depth points using the mask
      int j2=0;
      int w = 1024;
      int h = 544;
      for(int v=0; v<h; v=v+ decimate_) { // t2b
        for(int u=0; u<w; u=u+decimate_ ) {  //l2r
            if (mask_buf[v*w + u] > 0){ // if the mask is not black, apply it as red
              cloud3->points[j2].r = 255;//mask_buf[v*w + u];
              cloud3->points[j2].g = 0;//cloud3->points[j2].g/4;
              cloud3->points[j2].b = 0;//cloud3->points[j2].b/4; // reduce other color for emphaise
            }
            j2++;
        }
      }         
    }
    Isometry3dTime ref_pose_ROBOT_T = Isometry3dTime(current_utime, ref_pose_ROBOT);
    pc_vis_->pose_to_lcm_from_list(91002, ref_pose_ROBOT_T);  
    pc_vis_->ptcld_to_lcm_from_list(91003, *cloud3, current_utime, current_utime);      
  

    // 5. get depth image pixel values and store to file
    float* depths = depthImageView->getRangeImage()->getRangesArray();
    std::ofstream ofs("/tmp/depths.txt");
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        ofs << depths[i*width + j] << " ";
      }
      ofs << std::endl;
    }
    ofs.close();
    
    // 6. Reproject the depths into xyz and publish
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    for (int v = 0; v < height; ++v) { // rows t2b
      for (int u = 0; u < width; ++u) { // cols l2r
        pcl::PointXYZRGB pt;
        pt.z = 1/ depths[v*width + u]; /// inversion currently required due to Matt's interperation of depth as 1/depth ;)
        pt.x = ( pt.z * (u  - cx))/fx ;
        pt.y = ( pt.z * (v  - cy))/fy ;
        pt.r = 200.0; pt.g = 100.0; pt.b =100.0;
        cloud4->points.push_back(pt);
      }
    }
    Isometry3dTime ref_poseT = Isometry3dTime(current_utime, ref_pose);
    pc_vis_->pose_to_lcm_from_list(91004, ref_poseT);  
    pc_vis_->ptcld_to_lcm_from_list(91005, *cloud4, current_utime, current_utime);
    
    
  }
}



int main() {
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  
  // create state object instance
  State state(lcm );

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

  // start producing data
  DataProducer producer(&state);
  boost::thread producerThread(boost::ref(producer));

  // main lcm loop
  while (0 == state.mLcm->handle());

  // join pending threads
  producerThread.join();
}
