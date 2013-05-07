#include <mex.h>

#include <ctime>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>

#include <maps/BotWrapper.hpp>
#include <maps/Collector.hpp>
#include <maps/LocalMap.hpp>
#include <maps/MapManager.hpp>
#include <maps/ViewClient.hpp>
#include <maps/DepthImageView.hpp>

static const std::string kHeightMapChannel("MAP_CONTROL_HEIGHT");
static const int kHeightMapViewId = 1000;

class LcmLoop {
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::thread mThread;
  bool mRunning;

public:
  LcmLoop(const boost::shared_ptr<lcm::LCM>& iLcm) {
    mLcm = iLcm;
    mRunning = false;
  }

  ~LcmLoop() {
    stop();
  }

  void start() {
    if (mRunning) return;
    mRunning = true;
    mThread = boost::thread(boost::ref(*this));
  }

  void stop() {
    mRunning = false;
    mThread.join();
  }

  void operator()() {
    while(mRunning) {
      int fn = mLcm->getFileno();
      fd_set input_set;
      FD_ZERO(&input_set);
      FD_SET(fn, &input_set);
      struct timeval timeout = { 0, 200*1000 };
      int status = select(fn+1, &input_set, NULL, NULL, &timeout);
      if (status == 0) {
      }
      else if (status < 0) {
        mexPrintf("error in lcm pipe"); mexEvalString("drawnow");
        break;
      }
      else if (FD_ISSET(fn, &input_set)) {
        if (0 != mLcm->handle()) {
          mexPrintf("error in lcm handle\n"); mexEvalString("drawnow");
          break;
        }
      }
    }
  }
};

class MapGenerator {
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<maps::BotWrapper> mBotWrapper;
  boost::shared_ptr<maps::Collector> mCollector;
  boost::shared_ptr<maps::DepthImageView> mCurrentView;
  int mMapId;
  boost::thread mThread;
  bool mRunning;

public:
  MapGenerator(const boost::shared_ptr<lcm::LCM>& iLcm) {
    mLcm = iLcm;
    mRunning = false;
    mBotWrapper.reset(new maps::BotWrapper(mLcm, NULL, NULL));
    mCollector.reset(new maps::Collector());
    mCollector->setBotWrapper(mBotWrapper);

    // create new submap
    maps::LocalMap::Spec mapSpec;
    mapSpec.mId = 1;
    mMapId = mCollector->getMapManager()->createMap(mapSpec);

    // start running collector
    std::string laserChannel("SCAN_FREE");
    mCollector->getDataReceiver()->addChannel
      (laserChannel, maps::SensorDataReceiver::SensorTypePlanarLidar,
       laserChannel, "local");
    mCollector->start();
  }

  ~MapGenerator() {
    stop();
  }

  void start() {
    if (mRunning) return;
    mRunning = true;
    mThread = boost::thread(boost::ref(*this));
  }

  void stop() {
    mCollector->stop();
    mRunning = false;
    mThread.join();    
  }

  maps::DepthImageView::Ptr getView() const {
    return mCurrentView;
  }

  void operator()() {
    bool firstTime = true;
    while (mRunning) {

      // wait for timeout
      if (!firstTime) {
        const float mapPeriodSeconds = 1;
        boost::this_thread::sleep
          (boost::posix_time::milliseconds(mapPeriodSeconds*1000));
      }
      else {
        firstTime = false;
      }

      // get local map object
      auto manager = mCollector->getMapManager();
      auto localMap = manager->getMap(mMapId);
      if (localMap == NULL) {
        continue;
      }

      // constants
      const int64_t curTime = drc::Clock::instance()->getCurrentTime();
      const float resolutionX = 0.1;
      const float resolutionY = 0.1;
      const Eigen::Vector3f minPt(-2, -5, -3);
      const Eigen::Vector3f maxPt(5, 5, 0.3);
      const float timeWindowSeconds = 5;
      const int width = int((maxPt[0] - minPt[0]) / resolutionX);
      const int height = int((maxPt[1] - minPt[1]) / resolutionY);

      // transform for height map view
      Eigen::Isometry3f localToHead = Eigen::Isometry3f::Identity();
      if (!mBotWrapper->getTransform("local", "head", localToHead, curTime)) {
        continue;
      }
      Eigen::Isometry3f mapToHead = Eigen::Isometry3f::Identity();
      mapToHead.translation() = Eigen::Vector3f(0,0,100);
      mapToHead.linear() << 1,0,0,  0,-1,0,  0,0,-1;
      Eigen::Affine3f imageToMetric = Eigen::Affine3f::Identity();
      imageToMetric(0,0) = resolutionX;
      imageToMetric(1,1) = resolutionY;
      imageToMetric(0,3) = minPt[0];
      imageToMetric(1,3) = minPt[1];
      Eigen::Projective3f projector =
        imageToMetric.inverse() * mapToHead.inverse() * localToHead;

      // set up space-time bounds
      maps::LocalMap::SpaceTimeBounds bounds;
      bounds.mTimeMax = curTime;
      bounds.mTimeMin = curTime - timeWindowSeconds*1e6;
      bounds.mPlanes.push_back(Eigen::Vector4f( 1, 0, 0, -minPt[0]));
      bounds.mPlanes.push_back(Eigen::Vector4f(-1, 0, 0,  maxPt[0]));
      bounds.mPlanes.push_back(Eigen::Vector4f( 0, 1, 0, -minPt[1]));
      bounds.mPlanes.push_back(Eigen::Vector4f( 0,-1, 0,  maxPt[1]));
      bounds.mPlanes.push_back(Eigen::Vector4f( 0, 0, 1, -minPt[2]));
      bounds.mPlanes.push_back(Eigen::Vector4f( 0, 0,-1,  maxPt[2]));
      Eigen::Matrix4f planeTransform = localToHead.matrix().transpose();
      for (int k = 0; k < bounds.mPlanes.size(); ++k) {
        bounds.mPlanes[k] = planeTransform*bounds.mPlanes[k];
      }

      // get view
      mCurrentView =
        localMap->getAsDepthImage(width, height, projector, bounds);
      if (mCurrentView == NULL) {
        continue;
      }

      // set some view properties
      // 0 defaults to old behavior;
      // >0 sets radius (in pixels) for normal computation
      mCurrentView->setNormalRadius(0);

      // options are LeastSquares, RobustKernel, and SampleConsensus
      // this is ignored if radius==0
      mCurrentView->setNormalMethod
        (maps::DepthImageView::NormalMethodLeastSquares);
    }
  }
};

class ViewClientWrapper {
  maps::ViewClient mViewClient;
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<maps::BotWrapper> mBotWrapper;

public:
  ViewClientWrapper(const boost::shared_ptr<lcm::LCM>& iLcm) {
    mLcm = iLcm;
    mBotWrapper.reset(new maps::BotWrapper(mLcm, NULL, NULL));
    mViewClient.setBotWrapper(mBotWrapper);
    mViewClient.removeAllViewChannels();
    mViewClient.addViewChannel(kHeightMapChannel);
    mViewClient.start();
    requestHeightMap();
  }

  ~ViewClientWrapper() {
  }

  maps::DepthImageView::Ptr getView() const {
    maps::ViewBase::Ptr view = mViewClient.getView(kHeightMapViewId);
    return boost::static_pointer_cast<maps::DepthImageView>(view);
  }

  void requestHeightMap() {
    const Eigen::Vector3f minPt(-2, -5, -3);
    const Eigen::Vector3f maxPt(5, 5, 0.3);
    const float timeWindowSeconds = 5;

    maps::ViewBase::Spec spec;
    spec.mResolution = 0.1;
    spec.mWidth = int((maxPt[0] - minPt[0]) / spec.mResolution);
    spec.mHeight = int((maxPt[1] - minPt[1]) / spec.mResolution);
    spec.mTimeMin = -5*1e6;
    spec.mClipPlanes.push_back(Eigen::Vector4f( 1, 0, 0, -minPt[0]));
    spec.mClipPlanes.push_back(Eigen::Vector4f(-1, 0, 0,  maxPt[0]));
    spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 1, 0, -minPt[1]));
    spec.mClipPlanes.push_back(Eigen::Vector4f( 0,-1, 0,  maxPt[1]));
    spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 0, 1, -minPt[2]));
    spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 0,-1,  maxPt[2]));

    spec.mMapId = 1;
    spec.mViewId = kHeightMapViewId;
    spec.mType = maps::ViewBase::TypeDepthImage;
    spec.mChannel = kHeightMapChannel;
    spec.mFrequency = 1;
    spec.mTimeMax = 0;
    spec.mRelativeTime = true;
    spec.mRelativeLocation = true;
    spec.mActive = true;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.translation() = Eigen::Vector3f(0,0,10);
    pose.linear() << 1,0,0, 0,-1,0, 0,0,-1;
    Eigen::Affine3f calib = Eigen::Affine3f::Identity();
    calib(0,0) = 1/spec.mResolution;
    calib(1,1) = 1/spec.mResolution;
    calib(0,3) = -minPt[0]*calib(0,0);
    calib(1,3) = -minPt[1]*calib(1,1);
    Eigen::Projective3f projector = calib*pose.inverse();
    spec.mTransform = projector;
    mViewClient.request(spec);
  }
  
};

struct State {
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<LcmLoop> mLcmLoop;
  boost::shared_ptr<MapGenerator> mMapGenerator;
  boost::shared_ptr<ViewClientWrapper> mViewClientWrapper;

  State() {
    mLcm.reset(new lcm::LCM());
    mLcmLoop.reset(new LcmLoop(mLcm));
    drc::Clock::instance()->setLcm(mLcm);
    drc::Clock::instance()->setVerbose(false);

    if (false) {
      // TODO: not currently using this functionality
      mMapGenerator.reset(new MapGenerator(mLcm));
    }
    else {
      mViewClientWrapper.reset(new ViewClientWrapper(mLcm));
    }
  }

  ~State() {
  }

  maps::DepthImageView::Ptr getView() const {
    if (mMapGenerator != NULL) {
      return mMapGenerator->getView();
    }
    if (mViewClientWrapper != NULL) {
      return mViewClientWrapper->getView();
    }
    return maps::DepthImageView::Ptr();
  }

};


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  struct State* state = NULL;

  // no arguments; create wrapper
  if (nrhs == 0) {

    // create state
    state = new State();

    // start map generator running
    if (state->mMapGenerator != NULL) state->mMapGenerator->start();

    // start lcm running
    if (state->mLcmLoop != NULL) state->mLcmLoop->start();

    // return pointer to state object
    mxClassID cid;
    if (sizeof(state) == 4) cid = mxUINT32_CLASS;
    else if (sizeof(state) == 8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("DRC:MapWrapperRobot:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&state,sizeof(state));

    return;
  }


  // copy data handle to state pointer
  if (!mxIsNumeric(prhs[0]) || (mxGetNumberOfElements(prhs[0]) != 1)) {
    mexErrMsgIdAndTxt("DRC:MapWrapperRobot:BadInputs","the first argument should be the ptr");
  }
  memcpy(&state, mxGetData(prhs[0]), sizeof(state));

  // single argument (handle); destroy wrapper
  if (nrhs == 1) {
    delete state;
    return;
  } 

  maps::DepthImageView::Ptr view = state->getView();
  int n = mxGetN(prhs[1]);

  // called with null point array; return point cloud
  if (n == 0) {
    if (!view) {
      plhs[0] = mxCreateDoubleMatrix(3,0,mxREAL);
      return;
    }
    maps::PointCloud::Ptr cloud = view->getAsPointCloud();
    plhs[0] = mxCreateDoubleMatrix(3,cloud->size(),mxREAL);
    double* ptr = mxGetPr(plhs[0]);
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType pt = cloud->points[i];
      ptr[i*3+0] = pt.x;
      ptr[i*3+1] = pt.y;
      ptr[i*3+2] = pt.z;
    }
    return;
  }

  if (mxGetM(prhs[1]) != 3) {
    mexErrMsgIdAndTxt("DRC:mapAPIwrapper:BadInputs","pos must be 3xn\n");
  }
    
  plhs[0] = mxCreateDoubleMatrix(3,n,mxREAL);
  mxArray* pmxNormal = mxCreateDoubleMatrix(3,n,mxREAL);
    
  double* ppos = mxGetPr(prhs[1]);
  double* pclosest = mxGetPr(plhs[0]);
  double* pnormal = mxGetPr(pmxNormal);
  Eigen::Vector3f pos, closest, normal;

  for (int i = 0; i < n; ++i) {
    pos << ppos[3*i], ppos[3*i+1], ppos[3*i+2];
    if ((view != NULL) && view->getClosest(pos,closest,normal)) {
      for (int j = 0; j < 3; ++j) {
        pclosest[3*i+j] = closest[j];
        pnormal[3*i+j] = normal[j];
      }
    }
    else { // invalid view or off grid
      for (int j = 0; j < 3; ++j) {
        pclosest[3*i+j] = NAN;
        pnormal[3*i+j] = NAN;
      }
    }
  }
  
  if (nlhs > 1) plhs[1] = pmxNormal;
  else mxDestroyArray(pmxNormal);
}
