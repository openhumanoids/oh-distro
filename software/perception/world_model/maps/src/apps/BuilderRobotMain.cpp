#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <maps/DeltaPublisher.hpp>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>

#include <lcmtypes/drc/map_params_t.hpp>

#include <pcl/common/transforms.h>

#include <ConciseArgs>

using namespace std;

class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<DeltaPublisher> mDeltaPublisher;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  bool mTraceRays;
  string mMapParamsChannel;
  lcm::Subscription* mParamsSubscription;

  State() {
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mDeltaPublisher.reset(new DeltaPublisher());
    mManager.reset(new MapManager());
    mLcm.reset(new lcm::LCM());
    mSensorDataReceiver->setLcm(mLcm);
    mDeltaPublisher->setManager(mManager);
    mDeltaPublisher->setLcm(mLcm);

    // defaults; should be set by command line args in main()
    mTraceRays = false;
    mMapParamsChannel = "MAP_CREATE";
    mParamsSubscription = NULL;
  }

  ~State() {
    mLcm->unsubscribe(mParamsSubscription);
  }

  void onMapParams(const lcm::ReceiveBuffer* iBuf,
                   const std::string& iChannel,
                   const drc::map_params_t* iMessage) {
    std::cout << "Received map parameters; creating new map" << std::endl;
    mManager->setMapResolution(iMessage->resolution);
    mManager->setMapDimensions(Eigen::Vector3d(iMessage->dimensions[0],
                                               iMessage->dimensions[1],
                                               iMessage->dimensions[2]));
    Eigen::Isometry3d xform = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond quat;
    quat.x() = iMessage->transform_to_local.rotation.x;
    quat.y() = iMessage->transform_to_local.rotation.y;
    quat.z() = iMessage->transform_to_local.rotation.z;
    quat.w() = iMessage->transform_to_local.rotation.w;
    Eigen::Vector3d trans;
    trans[0] = iMessage->transform_to_local.translation.x;
    trans[1] = iMessage->transform_to_local.translation.y;
    trans[2] = iMessage->transform_to_local.translation.z;
    xform.rotate(quat);
    xform.translate(trans);
    mManager->createMap(xform, iMessage->map_id);
  }
};

class DataConsumer {
public:
  DataConsumer(State* iState) {
    mState = iState;
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
};

int main(const int iArgc, const char** iArgv) {
  // instantiate state object
  State state;

  // parse arguments
  double mapResolution = 0.02;
  string laserChannel = "ROTATING_SCAN";
  double xDim(10), yDim(10), zDim(10);
  int publishPeriod(3000);
  string paramsChannel = "MAP_PARAMS";
  string updateChannel = "MAP_UPDATE";
  string ackChannel = "MAP_ACK";
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(laserChannel, "l", "laser_channel",
          "laser channel to use in map creation");
  opt.add(xDim, "x", "xsize", "size of map in x direction");
  opt.add(yDim, "y", "ysize", "size of map in y direction");
  opt.add(zDim, "z", "zsize", "size of map in z direction");
  opt.add(state.mTraceRays, "t", "raytrace",
          "use raytracing when creating map");
  opt.add(mapResolution, "r", "resolution",
          "size of smallest (leaf) octree nodes");
  opt.add(publishPeriod, "p", "publish_period",
          "interval between delta publications, in ms");
  opt.add(paramsChannel, "c", "create_channel",
          "channel for publishing map create messages");
  opt.add(updateChannel, "u", "update_channel",
          "channel for publishing map update messages");
  opt.add(ackChannel, "a", "ack_channel",
          "channel for receiving ack messages");
  opt.parse();
  state.mSensorDataReceiver->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");
  state.mManager->setMapResolution(mapResolution);
  state.mManager->setMapDimensions(Eigen::Vector3d(xDim, yDim, zDim));
  state.mDeltaPublisher->setPublishInterval(publishPeriod);
  state.mDeltaPublisher->setParamsChannel(paramsChannel);
  state.mDeltaPublisher->setUpdateChannel(updateChannel);
  state.mDeltaPublisher->setAckChannel(ackChannel);

  // set up remaining parameters
  state.mSensorDataReceiver->setMaxBufferSize(100);
  state.mManager->setDataBufferLength(1000);
  state.mManager->createMap(Eigen::Isometry3d::Identity());
  state.mParamsSubscription =
    state.mLcm->subscribe(state.mMapParamsChannel,
                          &State::onMapParams, &state);

  // start running data receiver
  BotParam* theParam =
    bot_param_new_from_server(state.mLcm->getUnderlyingLCM(), 0);
  state.mSensorDataReceiver->setBotParam(theParam);
  state.mSensorDataReceiver->start();

  // start running the delta publisher
  state.mDeltaPublisher->start();

  // start consuming data
  DataConsumer consumer(&state);
  boost::thread thread(consumer);

  // main lcm loop
  while (0 == state.mLcm->handle());

  state.mDeltaPublisher->stop();
  thread.join();

  return 0;
}
