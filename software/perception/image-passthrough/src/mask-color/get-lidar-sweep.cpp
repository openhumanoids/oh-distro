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
#include <maps/BotWrapper.hpp>

#include <drc_utils/Clock.hpp>
#include <lcmtypes/bot_core.hpp>

using namespace maps;
using namespace std;

class State {
public:
  BotWrapper::Ptr mBotWrapper;
  boost::shared_ptr<Collector> mCollector;
  int mActiveMapId;
  bot_lcmgl_t* mLcmGl;
  
  State( boost::shared_ptr<lcm::LCM> &mLcm ) {
    mBotWrapper.reset(new BotWrapper(mLcm));
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
//    mCollector->setLcm(mLcm);
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
    
  protected:
    State* mState;
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   

    int64_t last_sweep_time_;
    std::string image_channel_;
    bool getSweep();
    
    // Point Cloud of most recent sweep:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, State* iState):
    lcm_(lcm_), image_channel_(image_channel_), mState(iState){

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_ = cloud_ptr;  
}


bool Pass::getSweep(){
  cout << "doing getSweep\n";
  
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
    cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | repeat\n";
    return false; 
  }
  last_sweep_time_ = timeMin;

  cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | process\n";
  LocalMap::SpaceTimeBounds bounds;
  bounds.mTimeMin = timeMin;
  bounds.mTimeMax = timeMax;

  // get and publish point cloud corresponding to this time range
  // (for debugging)
//  maps::PointCloud::Ptr cloud = localMap->getAsPointCloud(0, bounds);
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

  return true;
}


void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  getSweep();
}

int main() {
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  string channel = "CAMERALEFT";
  
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

  Pass app(lcm,channel, &state);  
  // main lcm loop
  while(0 == lcm->handle());
}
