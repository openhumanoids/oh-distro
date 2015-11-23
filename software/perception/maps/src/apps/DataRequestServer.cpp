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
#include <drc_utils/BotWrapper.hpp>

#include "RobotState.hpp"

using namespace std;

struct Worker {
  typedef std::shared_ptr<Worker> Ptr;

  bool mActive;
  drc::data_request_t mRequest;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::shared_ptr<maps::RobotState> mRobotState;
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
    if (mThread.joinable()) mThread.join();
  }

  void operator()() {
    mActive = true;
    int numCycles = 0;
    while (mActive) {

      // see if this is a cancel request
      if ((mRequest.period<0) || ((mRequest.period == 0) && (numCycles>0))) {
        mActive = false;
        break;
      }

      // dispatch
      switch(mRequest.type) {
      case drc::data_request_t::CAMERA_IMAGE_HEAD_LEFT:
      case drc::data_request_t::CAMERA_IMAGE_HEAD_RIGHT:
      case drc::data_request_t::CAMERA_IMAGE_LHAND:
      case drc::data_request_t::CAMERA_IMAGE_RHAND:
      case drc::data_request_t::CAMERA_IMAGE_LCHEST:
      case drc::data_request_t::CAMERA_IMAGE_RCHEST:
        sendCameraImageRequest(); break;
      case drc::data_request_t::MINIMAL_ROBOT_STATE:
        sendMinimalRobotStateRequest(); break;
//      case drc::data_request_t::AFFORDANCE_LIST:
  //      sendAffordanceListRequest(); break;
      case drc::data_request_t::MAP_CATALOG:
        sendMapViewCatalogRequest(); break;
      case drc::data_request_t::OCTREE_SCENE:
        sendOctreeSceneRequest(); break;
      case drc::data_request_t::OCTREE_WORKSPACE:
        sendOctreeWorkspaceRequest(); break;
      case drc::data_request_t::CLOUD_SCENE:
        sendCloudSceneRequest(); break;
      case drc::data_request_t::CLOUD_WORKSPACE:
        sendCloudWorkspaceRequest(); break;
      case drc::data_request_t::HEIGHT_MAP_SCENE:
        sendHeightMapSceneRequest(); break;
      case drc::data_request_t::HEIGHT_MAP_CORRIDOR:
        sendHeightMapCorridorRequest(); break;
      case drc::data_request_t::HEIGHT_MAP_COARSE:
        sendHeightMapCoarseRequest(); break;
      case drc::data_request_t::HEIGHT_MAP_DENSE:
        sendHeightMapDenseRequest(); break;
      case drc::data_request_t::DEPTH_MAP_SCENE:
        sendDepthMapSceneRequest(); break;
      case drc::data_request_t::DEPTH_MAP_WORKSPACE_C:
        sendDepthMapWorkspaceRequestCenter(); break;
      case drc::data_request_t::DEPTH_MAP_WORKSPACE_L:
        sendDepthMapWorkspaceRequestLeft(); break;
      case drc::data_request_t::DEPTH_MAP_WORKSPACE_R:
        sendDepthMapWorkspaceRequestRight(); break;
      case drc::data_request_t::DENSE_CLOUD_LHAND:
        sendDenseCloudLeftHandRequest(); break;
      case drc::data_request_t::DENSE_CLOUD_RHAND:
        sendDenseCloudRightHandRequest(); break;
      case drc::data_request_t::SCANS_HALF_SWEEP:
        sendScansHalfSweep(); break;
      case drc::data_request_t::TERRAIN_COST:
        sendTerrainCostRequest(); break;
      case drc::data_request_t::FUSED_DEPTH:
        sendFusedDepthRequest(); break;
      case drc::data_request_t::FUSED_HEIGHT:
        sendFusedHeightRequest(); break;
      case drc::data_request_t::STEREO_HEIGHT:
        sendStereoHeightRequest(); break;
      default:
        cout << "Unknown request type" << endl; break;
      }

      // wait for timer expiry
      // period is in tenths of seconds, so conversion to milli = x100
      ++numCycles;
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

/*
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
      affPlus.npoints = affPlus.ntriangles = 0;
      affordances[i]->aff->toMsg(&affPlus.aff);
      affPlus.aff.aff_store_control = drc::affordance_t::UPDATE;
      msg.affs_plus.push_back(affPlus);
    }
    mLcm->publish(affordance::AffordanceServer::
                  AFFORDANCE_PLUS_BASE_OVERWRITE_CHANNEL, &msg);
    cout << "Sent affordance list" << endl;
  }
*/

  void sendMapViewCatalogRequest() {
    mLcm->publish("TRIGGER_MAP_CATALOG", &mRequest);
    cout << "Sent map catalog" << endl;
  }

  void sendOctreeSceneRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.view_id = drc::data_request_t::OCTREE_SCENE;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendOctreeWorkspaceRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.map_id = 1; // 1 means use SCAN_FREE | 2 or 3 means use SCAN
    msg.view_id = drc::data_request_t::OCTREE_WORKSPACE;
    msg.resolution = 0.01;
    // Mode recently changed to time history
    //msg.time_min = 0;
    //msg.time_max = 180;
    //msg.time_mode = drc::map_request_t::ROLL_ANGLE_ABSOLUTE;
    msg.clip_planes[0][3] = 0.25;
    msg.clip_planes[1][3] = 2;
    msg.clip_planes[2][3] = 1;
    msg.clip_planes[3][3] = 1;
    msg.clip_planes[4][3] = 3;
    msg.clip_planes[5][3] = 0.3;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendCloudSceneRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.type = drc::map_request_t::POINT_CLOUD;
    msg.view_id = drc::data_request_t::CLOUD_SCENE;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendCloudWorkspaceRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.type = drc::map_request_t::POINT_CLOUD;
    msg.map_id = 2;
    msg.view_id = drc::data_request_t::CLOUD_WORKSPACE;
    msg.resolution = 0.01;
    msg.quantization_max = 0.01;
    msg.time_min = 0;
    msg.time_max = 180;
    msg.time_mode = drc::map_request_t::ROLL_ANGLE_ABSOLUTE;
    msg.clip_planes[0][3] = 0.25;
    msg.clip_planes[1][3] = 2;
    msg.clip_planes[2][3] = 1;
    msg.clip_planes[3][3] = 1;
    msg.clip_planes[4][3] = 3;
    msg.clip_planes[5][3] = 0.3;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendHeightMapSceneRequest() {
    const Eigen::Vector3f minPt(-1, -2, -3);
    const Eigen::Vector3f maxPt(5, 2, 0.3);
    drc::map_request_t msg =
      prepareHeightRequestMessage(minPt, maxPt, 0.03, 0.03);
    Eigen::Isometry3f pelvisPose;
    bool isProne = true;
    if (mBotWrapper->getTransform("body","local",pelvisPose)) {
      float zPelvis = pelvisPose(2,2);
      zPelvis = std::min(std::max(-1.0f,zPelvis),1.0f);  // clamp to [-1,1]
      const float kPi = acos(-1);
      const float angleThresh = 45;
      isProne = (acos(zPelvis) > angleThresh*kPi/180);
    }
    if (isProne) {
      msg.clip_planes[5][3] = 0;
    }
    else {
      Eigen::Vector4f plane(0.1, 0, -1, -0.4);
      plane /= plane.head<3>().norm();
      for (int k = 0; k < 4; ++k) msg.clip_planes[5][k] = plane[k];
    }
    msg.view_id = drc::data_request_t::HEIGHT_MAP_SCENE;
    msg.time_min = -5;
    msg.time_max = 185;
    msg.time_mode = drc::map_request_t::ROLL_ANGLE_ABSOLUTE;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendHeightMapCorridorRequest() {
    const Eigen::Vector3f minPt(0, -1, -3);
    const Eigen::Vector3f maxPt(10, 1, 0.3);
    drc::map_request_t msg =
      prepareHeightRequestMessage(minPt, maxPt, 0.05, 0.05);
    msg.view_id = drc::data_request_t::HEIGHT_MAP_CORRIDOR;
    msg.time_min = -10*1e6;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendHeightMapCoarseRequest() {
    const Eigen::Vector3f minPt(-5, -20, -3);
    const Eigen::Vector3f maxPt(30, 20, 0.3);
    drc::map_request_t msg =
      prepareHeightRequestMessage(minPt, maxPt, 0.5, 0.5);
    msg.view_id = drc::data_request_t::HEIGHT_MAP_COARSE;
    msg.time_min = -10*1e6;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendHeightMapDenseRequest() {
    const Eigen::Vector3f minPt(-5, -20, -3);
    const Eigen::Vector3f maxPt(30, 20, 0.3);
    drc::map_request_t msg =
      prepareHeightRequestMessage(minPt, maxPt, 0.1, 0.1);
    msg.view_id = drc::data_request_t::HEIGHT_MAP_DENSE;
    msg.time_min = -20*1e6;
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

  void sendDepthMapWorkspaceRequestNarrow(const float iYaw, const int iId) {
    drc::map_request_t msg = prepareRequestMessage();
    msg.map_id = 3;
    msg.view_id = iId;
    msg.resolution = 0.01;
    msg.width = 200;
    msg.height = 200;
    msg.quantization_max = 0.02;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    for (int i = 0; i < 6; ++i) msg.clip_planes[i][3] = 2;
    msg.clip_planes[0][3] = 0;
    msg.clip_planes[5][3] = 1;
    Eigen::Projective3f projector =
      createProjector(75, 110, msg.width, msg.height);
    const float kPi = acos(-1);
    Eigen::AngleAxisf angleAxis(-iYaw*kPi/180, Eigen::Vector3f(0,0,1));
    projector = projector*angleAxis;
    angleAxis = Eigen::AngleAxisf(-20*kPi/180, Eigen::Vector3f(1,0,0));
    projector = projector*angleAxis;
    setTransform(projector, msg);
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendDepthMapWorkspaceRequestCenter() {
    return sendDepthMapWorkspaceRequest();
    // This request was not working so
    // I went back to the original full frontal view
    //return sendDepthMapWorkspaceRequestNarrow
    //  (0,drc::data_request_t::DEPTH_MAP_WORKSPACE_C);
  }

  void sendDepthMapWorkspaceRequestLeft() {
    return sendDepthMapWorkspaceRequestNarrow
      (40,drc::data_request_t::DEPTH_MAP_WORKSPACE_L);
  }

  void sendDepthMapWorkspaceRequestRight() {
    return sendDepthMapWorkspaceRequestNarrow
      (-40,drc::data_request_t::DEPTH_MAP_WORKSPACE_R);
  }


  void sendDepthMapWorkspaceRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.map_id = 1; // 2 or 3 means use SCAN | 1 means use SCAN_FREE
    msg.view_id = drc::data_request_t::DEPTH_MAP_WORKSPACE_C;
    msg.resolution = 0.01;
    msg.width = msg.height = 200;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    for (int i = 0; i < 6; ++i) {
      msg.clip_planes[i][3] = 2;
    }
    msg.clip_planes[0][3] = 0.5;
    msg.clip_planes[5][3] = 1;
    Eigen::Projective3f projector =
      createProjector(100, 90, msg.width, msg.height,
                      Eigen::Vector3f(-0.5,0,0));
    setTransform(projector, msg);
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendDenseCloudLeftHandRequest() {
    Eigen::Vector3f handPos(0,0,0);
    Eigen::Quaternionf dummy;
    if (!mRobotState->getPose("l_hand", dummy, handPos)) return;
    drc::map_request_t msg = getDenseCloudBoxRequest(handPos, 0.25, 10);
    msg.map_id = 2;
    msg.view_id = drc::data_request_t::DENSE_CLOUD_LHAND;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendDenseCloudRightHandRequest() {
    Eigen::Vector3f handPos(0,0,0);
    Eigen::Quaternionf dummy;
    if (!mRobotState->getPose("r_hand", dummy, handPos)) return;
    drc::map_request_t msg = getDenseCloudBoxRequest(handPos, 0.25, 10);
    msg.map_id = 2;
    msg.view_id = drc::data_request_t::DENSE_CLOUD_RHAND;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  drc::map_request_t getDenseCloudBoxRequest(const Eigen::Vector3f& iPos,
                                             const float iSize,
                                             const float iTimeWindow) {
    Eigen::Vector3f boxMin = iPos-Eigen::Vector3f(1,1,1)*iSize;
    Eigen::Vector3f boxMax = iPos+Eigen::Vector3f(1,1,1)*iSize;
    drc::map_request_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.map_id = 1;
    msg.view_id = 1;
    msg.type = drc::map_request_t::POINT_CLOUD;
    msg.resolution = 0.01;
    msg.frequency = 0;
    msg.quantization_max = 0.01;
    msg.time_min = -iTimeWindow*1e6;
    msg.time_max = 0;
    msg.time_mode = drc::map_request_t::RELATIVE;
    msg.relative_location = false;
    msg.clip_planes.push_back(std::vector<float>({ 1, 0, 0, -boxMin[0]}));
    msg.clip_planes.push_back(std::vector<float>({-1, 0, 0,  boxMax[0]}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 1, 0, -boxMin[1]}));
    msg.clip_planes.push_back(std::vector<float>({ 0,-1, 0,  boxMax[1]}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 0, 1, -boxMin[2]}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 0,-1,  boxMax[2]}));
    msg.num_clip_planes = msg.clip_planes.size();
    msg.active = true;
    msg.width = msg.height = 0;
    setTransform(Eigen::Projective3f::Identity(), msg);
    return msg;
  }

  void sendScansHalfSweep() {
    drc::map_request_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.map_id = 2;
    msg.view_id = drc::data_request_t::SCANS_HALF_SWEEP;
    msg.type = drc::map_request_t::SCAN_BUNDLE;
    msg.resolution = 0.005;
    msg.frequency = 0;
    msg.quantization_max = 0.005;
    msg.time_min = -3;
    msg.time_max = 182;
    msg.time_mode = drc::map_request_t::ROLL_ANGLE_ABSOLUTE;
    msg.relative_location = false;
    msg.num_clip_planes = 0;
    msg.active = true;
    msg.width = msg.height = 0;
    setTransform(Eigen::Projective3f::Identity(), msg);
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendTerrainCostRequest() {
    drc::shaper_data_request_t msg;
    msg.channel = "TERRAIN_DIST_MAP";
    msg.priority = 1;
    mLcm->publish("SHAPER_DATA_REQUEST", &msg);
  }

  void sendFusedDepthRequest() {
    drc::map_request_t msg = prepareRequestMessage();
    msg.view_id = drc::data_request_t::FUSED_DEPTH;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    msg.clip_planes.clear();
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendFusedHeightRequest() {
    const Eigen::Vector3f minPt(-1, -2, -3);
    const Eigen::Vector3f maxPt(5, 2, 0.3);
    auto msg = prepareHeightRequestMessage(minPt, maxPt, 0.01, 0.01);
    Eigen::Vector4f plane(0.1, 0, -1, -0.4);
    plane /= plane.head<3>().norm();
    for (int k = 0; k < 4; ++k) msg.clip_planes[5][k] = plane[k];
    msg.view_id = drc::data_request_t::FUSED_HEIGHT;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  void sendStereoHeightRequest() {
    const Eigen::Vector3f minPt(-1, -2, -3);
    const Eigen::Vector3f maxPt(5, 2, 0.3);
    auto msg = prepareHeightRequestMessage(minPt, maxPt, 0.02, 0.02);
    msg.view_id = drc::data_request_t::STEREO_HEIGHT;
    mLcm->publish("MAP_REQUEST", &msg);
  }

  Eigen::Projective3f createProjector(const float iHorzFovDegrees,
                                      const float iVertFovDegrees,
                                      const int iWidth, const int iHeight) {
    return createProjector(iHorzFovDegrees, iVertFovDegrees,
                           iWidth, iHeight, Eigen::Vector3f(0,0,0));
  }

  Eigen::Projective3f createProjector(const float iHorzFovDegrees,
                                      const float iVertFovDegrees,
                                      const int iWidth, const int iHeight,
                                      const Eigen::Vector3f& iPosition) {
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
    pose.translation() = -pose.linear()*iPosition;
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

  static void setTransform(const Eigen::Projective3f& iTransform,
                           drc::map_request_t& oMessage) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        oMessage.transform[i][j] = iTransform(i,j);
      }
    }
  }    

  drc::map_request_t
  prepareHeightRequestMessage(const Eigen::Vector3f& iMinPt,
                              const Eigen::Vector3f& iMaxPt,
                              const float iResX, const float iResY) {
    drc::map_request_t msg = prepareRequestMessage();
    msg.resolution = 0.5*(iResX + iResY);
    msg.width = int((iMaxPt[0] - iMinPt[0]) / iResX);
    msg.height = int((iMaxPt[1] - iMinPt[1]) / iResY);
    msg.accum_type = drc::map_request_t::ROBUST_BLEND;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    msg.clip_planes[0][3] = -iMinPt[0];
    msg.clip_planes[1][3] = iMaxPt[0];
    msg.clip_planes[2][3] = -iMinPt[1];
    msg.clip_planes[3][3] = iMaxPt[1];
    msg.clip_planes[4][3] = -iMinPt[2];
    msg.clip_planes[5][3] = iMaxPt[2];
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.translation() = Eigen::Vector3f(0,0,10);
    pose.linear() << 1,0,0, 0,-1,0, 0,0,-1;
    Eigen::Affine3f calib = Eigen::Affine3f::Identity();
    calib(0,0) = 1/iResX;
    calib(1,1) = 1/iResY;
    calib(0,3) = -iMinPt[0]*calib(0,0);
    calib(1,3) = -iMinPt[1]*calib(1,1);
    Eigen::Projective3f projector = calib*pose.inverse();
    setTransform(projector, msg);
    return msg;
  }

  drc::map_request_t prepareRequestMessage() {
    drc::map_request_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.map_id = 1;
    msg.view_id = 1;
    msg.type = drc::map_request_t::OCTREE;
    msg.resolution = 0.1;
    msg.frequency = 0;
    msg.quantization_max = 0.01;
    msg.time_min = -20*1e6;
    msg.time_max = 0;
    msg.time_mode = drc::map_request_t::RELATIVE;
    msg.relative_location = true;
    // this used to be CLOSEST, but wasn't working, so flipped it:
    msg.accum_type = drc::map_request_t::FURTHEST;
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
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::shared_ptr<maps::RobotState> mRobotState;
  typedef std::unordered_map<int,Worker::Ptr> WorkerMap;
  WorkerMap mWorkers;

  State() {
    mLcm.reset(new lcm::LCM());
    mBotWrapper.reset(new drc::BotWrapper(mLcm));
    mRobotState.reset(new maps::RobotState(mLcm));
    drc::Clock::instance()->setLcm(mLcm->getUnderlyingLCM());
    drc::Clock::instance()->setVerbose(false);
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
        worker->mBotWrapper = mBotWrapper;
        worker->mRobotState = mRobotState;
//        worker->mAffordanceWrapper = mAffordanceWrapper;
        mWorkers[req.type] = worker;
        item = mWorkers.find(req.type);
      }
      else {
        if ((req.period < 0) && item->second->mActive) {
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
