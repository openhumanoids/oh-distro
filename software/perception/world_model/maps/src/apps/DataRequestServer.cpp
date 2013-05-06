#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <unordered_map>

#include <Eigen/Geometry>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/data_request_list_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>
#include <lcmtypes/drc/shaper_data_request_t.hpp>

#include <ConciseArgs>
#include <drc_utils/Clock.hpp>
#include <affordance/AffordanceUpWrapper.h>

using namespace std;

struct Worker {
  typedef std::shared_ptr<Worker> Ptr;

  bool mActive;
  drc::data_request_t mRequest;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<affordance::AffordanceUpWrapper> mAffordanceWrapper;
  std::thread mThread;

  ~Worker() {
    stop();
  }

  void start() {
    if (mActive) return;
    if (mThread.joinable()) mThread.join();
    mActive = true;
    mThread = std::thread(std::ref(*this));
  }

  void stop() {
    mActive = false;
    mThread.join();
  }

  void operator()() {
    mActive = true;
    while (mActive) {

      switch(mRequest.type) {
      case drc::data_request_t::CAMERA_IMAGE:
        sendCameraImageRequest();
        break;
      case drc::data_request_t::MINIMAL_ROBOT_STATE:
        sendMinimalRobotStateRequest();
        break;
      case drc::data_request_t::AFFORDANCE_LIST:
        sendAffordanceListRequest();
        break;
      case drc::data_request_t::MAP_CATALOG:
        sendMapViewCatalogRequest();
        break;
      case drc::data_request_t::OCTREE_SCENE:
        sendOctreeSceneRequest();
        break;
      case drc::data_request_t::HEIGHT_MAP_SCENE:
        sendHeightMapSceneRequest();
        break;
      case drc::data_request_t::HEIGHT_MAP_CORRIDOR:
        sendHeightMapCorridorRequest();
        break;
      case drc::data_request_t::DEPTH_MAP_SCENE:
        sendDepthMapSceneRequest();
        break;
      case drc::data_request_t::DEPTH_MAP_WORKSPACE:
        sendDepthMapWorkspaceRequest();
        break;
      default:
        cout << "Unknown request type" << endl;
        break;
      }

      // see if this is just a one-shot request
      if (mRequest.period == 0) {
        mActive = false;
        break;
      }

      // wait for timer expiry
      // period is in tenths of seconds, so conversion to milli = x100
      int millis = mRequest.period*100;
      std::this_thread::sleep_for(std::chrono::milliseconds(millis));
    }
  }

  void sendCameraImageRequest() {
    mLcm->publish("TRIGGER_CAMERA", &mRequest);
    cout << "Sent camera image request" << endl;
  }

  void sendMinimalRobotStateRequest() {
    mLcm->publish("TRIGGER_STATE", &mRequest);
    cout << "Sent minimal robot state request" << endl;
  }

  void sendAffordanceListRequest() {
    std::vector<affordance::AffPlusPtr> affordances;
    mAffordanceWrapper->getAllAffordancesPlus(affordances);
    drc::affordance_plus_collection_t msg;
    msg.name = "updateFromDataRequestServer";
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.map_id = -1;
    msg.naffs = affordances.size();
    for (int i = 0; i < affordances.size(); ++i) {
      drc::affordance_plus_t affPlus;
      affordances[i]->toMsg(&affPlus);
      affPlus.aff.aff_store_control = drc::affordance_t::UPDATE;
      msg.affs_plus.push_back(affPlus);
    }
    mLcm->publish(affordance::AffordanceServer::
                  AFFORDANCE_PLUS_BASE_OVERWRITE_CHANNEL, &msg);
    cout << "Sent affordance list" << endl;
  }

  void sendMapViewCatalogRequest() {
    mLcm->publish("TRIGGER_MAP_CATALOG", &mRequest);
    cout << "Sent map catalog" << endl;
  }

  void sendOctreeSceneRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.view_id = drc::data_request_t::OCTREE_SCENE;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendHeightMapSceneRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.view_id = drc::data_request_t::HEIGHT_MAP_SCENE;
    msg.resolution = 0.05;
    msg.width = 140;
    msg.height = 200;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    msg.time_min = -5*1e6;
    msg.clip_planes[0][3] = 2;
    msg.clip_planes[4][3] = 3;
    msg.clip_planes[5][3] = 0.3;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.translation() = Eigen::Vector3f(0,0,10);
    pose.linear() << 1,0,0, 0,-1,0, 0,0,-1;
    Eigen::Affine3f calib = Eigen::Affine3f::Identity();
    calib(0,0) = msg.width/(msg.clip_planes[0][3] + msg.clip_planes[1][3]);
    calib(1,1) = msg.height/(msg.clip_planes[2][3] + msg.clip_planes[3][3]);
    calib(0,3) = msg.clip_planes[0][3]*calib(0,0);
    calib(1,3) = msg.clip_planes[2][3]*calib(1,1);
    Eigen::Projective3f projector = calib*pose.inverse();
    setTransform(projector, msg);
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendHeightMapCorridorRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.view_id = drc::data_request_t::HEIGHT_MAP_CORRIDOR;
    msg.resolution = 0.02;
    msg.width = msg.height = 100;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    msg.time_min = -5*1e6;
    msg.clip_planes[0][3] = 0;
    msg.clip_planes[1][3] = 10;
    msg.clip_planes[2][3] = 1;
    msg.clip_planes[3][3] = 1;
    msg.clip_planes[4][3] = 3;
    msg.clip_planes[5][3] = 0.3;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.translation() = Eigen::Vector3f(0,0,10);
    pose.linear() << 1,0,0, 0,-1,0, 0,0,-1;
    Eigen::Affine3f calib = Eigen::Affine3f::Identity();
    calib(0,0) = msg.width/(msg.clip_planes[0][3] + msg.clip_planes[1][3]);
    calib(1,1) = msg.height/(msg.clip_planes[2][3] + msg.clip_planes[3][3]);
    calib(0,3) = msg.clip_planes[0][3]*calib(0,0);
    calib(1,3) = msg.clip_planes[2][3]*calib(1,1);
    Eigen::Projective3f projector = calib*pose.inverse();
    setTransform(projector, msg);
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendDepthMapSceneRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.view_id = drc::data_request_t::DEPTH_MAP_SCENE;
    msg.resolution = 0.1;
    msg.width = msg.height = 200;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    msg.clip_planes[0][3] = 0;
    Eigen::Projective3f projector =
        createProjector(160, 90, msg.width, msg.height);
    setTransform(projector, msg);
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendDepthMapWorkspaceRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.view_id = drc::data_request_t::DEPTH_MAP_WORKSPACE;
    msg.resolution = 0.02;
    msg.width = msg.height = 200;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    for (int i = 0; i < 6; ++i) {
      msg.clip_planes[i][3] = 2;
    }
    msg.clip_planes[0][3] = 0;
    msg.clip_planes[5][3] = 1;
    Eigen::Projective3f projector =
        createProjector(100, 90, msg.width, msg.height);
    setTransform(projector, msg);
    mLcm->publish("MAP_REQUEST", &msg);
  }

  Eigen::Projective3f createProjector(const float iHorzFovDegrees,
                                      const float iVertFovDegrees,
                                      const int iWidth, const int iHeight) {
    const float degToRad = 4*atan(1)/180;
    const float hFov = iHorzFovDegrees*degToRad;
    const float vFov = iVertFovDegrees*degToRad;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    Eigen::Vector3f z(cos(vFov/2), 0, -sin(vFov/2));
    Eigen::Vector3f x = z.cross(Eigen::Vector3f::UnitZ()).normalized();
    Eigen::Vector3f y = z.cross(x).normalized();
    pose.linear().row(0) = x;
    pose.linear().row(1) = y;
    pose.linear().row(2) = z;
    //pose.linear() << 0,-1,0, 0,0,-1, 1,0,0;
    Eigen::Projective3f calib = Eigen::Projective3f::Identity();
    calib.matrix().row(2).swap(calib.matrix().row(3));
    calib(0,0) = iWidth/2/tan(hFov/2);
    calib(1,1) = iHeight/2/tan(vFov/2);
    calib(0,2) = iWidth/2;
    calib(1,2) = iHeight/2;
    Eigen::Projective3f projector = calib*pose;
    return projector;
  }

  void setTransform(const Eigen::Projective3f& iTransform,
                    drc::map_request_t& oMessage) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        oMessage.transform[i][j] = iTransform(i,j);
      }
    }
  }    

  drc::map_request_t prepareRequestMessage() {
    drc::map_request_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.map_id = 1;
    msg.view_id = 1;
    msg.type = drc::map_request_t::OCTREE;
    msg.resolution = 0.1;
    msg.frequency = 0;
    msg.time_min = -20*1e6;
    msg.time_max = 0;
    msg.relative_time = true;
    msg.relative_location = true;
    msg.clip_planes.push_back(std::vector<float>({ 1, 0, 0, 5}));
    msg.clip_planes.push_back(std::vector<float>({-1, 0, 0, 5}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 1, 0, 5}));
    msg.clip_planes.push_back(std::vector<float>({ 0,-1, 0, 5}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 0, 1, 5}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 0,-1, 5}));
    msg.num_clip_planes = msg.clip_planes.size();
    msg.active = true;
    msg.width = msg.height = 0;
    setTransform(Eigen::Projective3f::Identity(), msg);
    return msg;
  }
};

struct State {
  std::shared_ptr<lcm::LCM> mLcm;
  typedef std::unordered_map<int,Worker::Ptr> WorkerMap;
  WorkerMap mWorkers;
  std::shared_ptr<affordance::AffordanceUpWrapper> mAffordanceWrapper;

  State() {
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm->getUnderlyingLCM());
    drc::Clock::instance()->setVerbose(false);

    boost::shared_ptr<lcm::LCM>
      boostLcm(new lcm::LCM(mLcm->getUnderlyingLCM()));
    mAffordanceWrapper.reset(new affordance::AffordanceUpWrapper(boostLcm));
  }

  ~State() {
  }

  void onDataRequest(const lcm::ReceiveBuffer* iBuf,
                     const std::string& iChannel,
                     const drc::data_request_list_t* iMessage) {
    cout << "Received request of size " << int(iMessage->num_requests) << endl;
    for (int i = 0; i < iMessage->num_requests; ++i) {
      const drc::data_request_t& req = iMessage->requests[i];
      WorkerMap::const_iterator item = mWorkers.find(req.type);
      if (item == mWorkers.end()) {
        Worker::Ptr worker(new Worker());
        worker->mActive = false;
        worker->mLcm = mLcm;
        worker->mAffordanceWrapper = mAffordanceWrapper;
        mWorkers[req.type] = worker;
        item = mWorkers.find(req.type);
      }
      else {
        if (req.period == 0 && item->second->mActive) {
          item->second->stop();
          continue;
        }
      }
      item->second->mRequest = req;
      item->second->start();
    }
  }
};

int main(const int iArgc, const char** iArgv) {
  State state;

  // parse arguments
  string requestChannel = "DATA_REQUEST";
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(requestChannel, "r", "request_channel",
          "channel for incoming data requests");
  opt.parse();

  // subscribe to channels
  state.mLcm->subscribe(requestChannel, &State::onDataRequest, &state);

  cout << "Waiting for input..." << endl;

  // main lcm loop
  while (0 == state.mLcm->handle());

  return 0;
}
