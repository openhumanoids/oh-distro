#include <mex.h>

#include <ctime>
#include <memory>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>

#include <maps/BotWrapper.hpp>
#include <maps/ViewClient.hpp>
#include <maps/DepthImageView.hpp>

static const std::string kHeightMapChannel("MAP_CONTROL_HEIGHT");
static const int kHeightMapViewId = 1000;

class LcmLoop {
  std::shared_ptr<lcm::LCM> mLcm;
  std::thread mThread;
  bool mRunning;

public:
  LcmLoop(const std::shared_ptr<lcm::LCM>& iLcm) {
    mLcm = iLcm;
    mRunning = false;
  }

  ~LcmLoop() {
    stop();
  }

  void start() {
    if (mRunning) return;
    mRunning = true;
    mThread = std::thread(std::ref(*this));
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

class ViewClientWrapper {
  maps::ViewClient mViewClient;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<maps::BotWrapper> mBotWrapper;

public:
  ViewClientWrapper(const std::shared_ptr<lcm::LCM>& iLcm) {
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
    return std::static_pointer_cast<maps::DepthImageView>(view);
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
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<LcmLoop> mLcmLoop;
  std::shared_ptr<ViewClientWrapper> mViewClientWrapper;

  State() {
    mLcm.reset(new lcm::LCM());
    mLcmLoop.reset(new LcmLoop(mLcm));
    drc::Clock::instance()->setLcm(mLcm);
    drc::Clock::instance()->setVerbose(false);
    mViewClientWrapper.reset(new ViewClientWrapper(mLcm));
  }

  ~State() {
  }

  maps::DepthImageView::Ptr getView() const {
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
  if (view != NULL) {
    view->setNormalRadius(2);
    view->setNormalMethod(maps::DepthImageView::NormalMethodLeastSquares);
  }
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
