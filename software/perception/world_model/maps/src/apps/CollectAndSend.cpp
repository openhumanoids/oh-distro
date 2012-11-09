#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>

#include <lcmtypes/drc/local_map_t.hpp>
#include <lcmtypes/drc/heightmap_t.hpp>
#include <bot_core/timestamp.h>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <pcl/common/transforms.h>

#include <ConciseArgs>

using namespace std;

class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  bool mTraceRays;
  int mPublishPeriod;
  string mMapChannel;

  State() {
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mManager.reset(new MapManager());
    mLcm.reset(new lcm::LCM());
    mSensorDataReceiver->setLcm(mLcm);

    // defaults; should be set by command line args in main()
    mTraceRays = false;
    mPublishPeriod = 3000;
    mMapChannel = "LOCAL_MAP";
  }

  ~State() {
  }
};

class DataConsumer {
public:
  DataConsumer(State* iState) {
    mState = iState;
    mCounter = 0;
  }

  void operator()() {
    while(true) {
      SensorDataReceiver::PointCloudWithPose data;
      if (mState->mSensorDataReceiver->waitForData(data)) {
        mState->mManager->addToBuffer(data.mTimestamp, data.mPointCloud,
                                      data.mPose);
        mState->mManager->getActiveMap()->add(data.mPointCloud, data.mPose,
                                              mState->mTraceRays);
      }
    }
  }

protected:
  State* mState;
  int mCounter;
};

class DataPublisher {
public:
  DataPublisher(State* iState) {
    mState = iState;
  }
  void operator()() {
    while(true) {
      // wait for timer expiry
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      timer.expires_from_now(boost::posix_time::
                             milliseconds(mState->mPublishPeriod));
      timer.wait();

      // see if map exists
      boost::shared_ptr<LocalMap> localMap = mState->mManager->getActiveMap();
      if (localMap == NULL) {
        continue;
      }

      // publish as local map
      std::vector<char> bytes;
      localMap->setStateId(localMap->getStateId()+1);
      localMap->serialize(bytes);
      drc::local_map_t mapMessage;
      mapMessage.utime = bot_timestamp_now();
      mapMessage.id = localMap->getId();
      mapMessage.state_id = localMap->getStateId();
      mapMessage.size_bytes = bytes.size();
      mapMessage.data.insert(mapMessage.data.end(), bytes.begin(), bytes.end());
      mState->mLcm->publish(mState->mMapChannel, &mapMessage);
      cout << "Published local map (" << bytes.size() << " bytes)" << endl;

      // publish as octomap
      std::cout << "Publishing octomap..." << std::endl;
      octomap::raw_t raw = mState->mManager->getActiveMap()->getAsRaw();
      mState->mLcm->publish("OCTOMAP", &raw);
    }
  }

protected:
  State* mState;
};

int main(const int iArgc, const char** iArgv) {
  // instantiate state object
  State state;

  // parse arguments
  double mapResolution = 0.02;
  string laserChannel = "ROTATING_SCAN";
  double xDim(10), yDim(10), zDim(10);
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(state.mMapChannel, "m", "map_channel",
          "channel to publish local maps");
  opt.add(laserChannel, "l", "laser", "laser channel to use in map creation");
  opt.add(xDim, "x", "xsize", "size of map in x direction");
  opt.add(yDim, "y", "ysize", "size of map in y direction");
  opt.add(zDim, "z", "zsize", "size of map in z direction");
  opt.add(state.mTraceRays, "r", "raytrace",
          "use raytracing when creating map");
  opt.add(state.mPublishPeriod, "p", "publish_period",
          "interval between map publications, in ms");
  opt.parse();
  state.mSensorDataReceiver->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");
  state.mManager->setMapResolution(mapResolution);
  state.mManager->setMapDimensions(Eigen::Vector3d(xDim, yDim, zDim));
  
  // set up remaining parameters
  state.mSensorDataReceiver->setMaxBufferSize(100);
  state.mManager->setDataBufferLength(1000);
  state.mManager->createMap(Eigen::Isometry3d::Identity());

  // start running data receiver
  BotParam* theParam =
    bot_param_new_from_server(state.mLcm->getUnderlyingLCM(), 0);
  state.mSensorDataReceiver->setBotParam(theParam);
  state.mSensorDataReceiver->start();

  // start consuming data
  DataConsumer consumer(&state);
  boost::thread thread(consumer);

  // start publishing data
  DataPublisher publisher(&state);
  boost::thread publishThread(publisher);

  // handle lcm events
  while (0 == state.mLcm->handle());

  thread.join();
  publishThread.join();

  return 0;
}
