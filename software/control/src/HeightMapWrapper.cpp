#include <mex.h>

#include <ctime>
#include <memory>
#include <thread>
#include <unordered_map>
#include <sstream>

#include <Eigen/Sparse>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>

#include <lcmtypes/drc/data_request_t.hpp>

#include <maps/BotWrapper.hpp>
#include <maps/ViewClient.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/DepthImage.hpp>

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
    mexPrintf("started lcm loop\n"); mexEvalString("drawnow");
  }

  void stop() {
    mRunning = false;
    if (mThread.joinable()) mThread.join();
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
        mexPrintf("error in lcm pipe\n"); mexEvalString("drawnow");
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
  int mHeightMapViewId;

public:
  ViewClientWrapper(const std::shared_ptr<lcm::LCM>& iLcm) {
    mLcm = iLcm;
    mBotWrapper.reset(new maps::BotWrapper(mLcm, NULL, NULL));
    mViewClient.setBotWrapper(mBotWrapper);
    setHeightMapChannel(kHeightMapChannel, kHeightMapViewId);
  }

  ~ViewClientWrapper() {
    stop();
  }

  void setHeightMapChannel(const std::string& iChannel, const int iViewId) {
    mViewClient.removeAllViewChannels();
    mViewClient.addViewChannel(iChannel);
    mHeightMapViewId = iViewId;
  }

  bool start() {
    mViewClient.start();
    requestHeightMap();
    return true;
  }

  bool stop() {
    mViewClient.stop();
    return true;
  }

  maps::DepthImageView::Ptr getView() {
    maps::ViewBase::Ptr view = mViewClient.getView(mHeightMapViewId);
    return std::static_pointer_cast<maps::DepthImageView>(view);
  }

  void requestHeightMap() {
    if (mHeightMapViewId != kHeightMapViewId) {
      return;
    }
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
  int mId;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<ViewClientWrapper> mViewClientWrapper;

  int mNormalRadius;
  bool mShouldFill;
  int64_t mLastUpdateTime;

  State(const int iId, const std::shared_ptr<lcm::LCM>& iLcm) {
    mId = iId;
    mLcm = iLcm;
    mNormalRadius = 0;
    mLastUpdateTime = 0;
    mShouldFill = false;
    mViewClientWrapper.reset(new ViewClientWrapper(mLcm));
    mexPrintf("created state object\n"); mexEvalString("drawnow");
  }

  ~State() {
    mexPrintf("destroying state object\n"); mexEvalString("drawnow");
    stop();
  }

  void start() {
    mViewClientWrapper->start();
  }

  void stop() {
    mViewClientWrapper->stop();
  }

  void fillView(maps::DepthImageView::Ptr& iView) {
    maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
    maps::DepthImage::Ptr img = iView->getDepthImage();
    std::vector<float>& depths =
      const_cast<std::vector<float>&>(img->getData(type));

    // compute max z change
    Eigen::Projective3f mapToWorld = iView->getTransform().inverse();
    Eigen::Vector4f p0 = mapToWorld*Eigen::Vector4f(0,0,0,1);
    Eigen::Vector4f px = mapToWorld*Eigen::Vector4f(1,0,0,1);
    float dx = (px-p0).norm();
    const float kPi = std::acos(-1.0);
    float maxDiffZ = tan(45*kPi/180)*dx;

    // build invalid sample list
    const float invalidValue = img->getInvalidValue(type);
    std::vector<int> invalidMap(depths.size());
    std::vector<bool> pixelsToUpdate(depths.size());
    std::vector<int> invalidList;
    invalidList.reserve(depths.size());
    std::fill(invalidMap.begin(), invalidMap.end(), -1);
    std::fill(pixelsToUpdate.begin(), pixelsToUpdate.end(), false);
    int w(img->getWidth()), h(img->getHeight());
    for (int i = 0, idx = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j, ++idx) {
        bool shouldUpdatePixel = false;
        bool invalidPixel = false;
        float z = depths[idx];
        if (z == invalidValue) shouldUpdatePixel = invalidPixel = true;
        else {
          if ((i == h-1) || (j == w-1)) invalidPixel = true;
          else {
            float zx(depths[idx+1]), zy(depths[idx+w]);
            if ((zx == invalidValue) || (zy == invalidValue)) {
              invalidPixel = true;
            }
            else {
              float gx(fabs(zx-z)), gy(fabs(zy-z));
              if ((gx > maxDiffZ) || (gy > maxDiffZ)) invalidPixel = true;
            }
          }
        }
        if (invalidPixel) { 
          invalidMap[idx] = invalidList.size();
          invalidList.push_back(idx);
          if (shouldUpdatePixel) pixelsToUpdate[idx] = true;
        }
      }
    }
    if (invalidList.size() == 0) return;

    // set up sparse linear system using 4-way neighbors
    std::vector<Eigen::Triplet<float> > triplets;
    triplets.reserve(invalidList.size()*5);
    Eigen::VectorXf rhs(invalidList.size());
    rhs.setZero();
    int xMax(w-1), yMax(h-1);
    for (size_t i = 0; i < invalidList.size(); ++i) {
      int index = invalidList[i];
      int x = index%w;
      int y = index/w;
      bool left(x==0), right(x==xMax), top(y==0), bottom(y==yMax);
      float numNeighbors = int(!left) + int(!right) + int(!top) + int(!bottom);
      triplets.push_back(Eigen::Triplet<float>(i,i,numNeighbors));
      if (!left) {
        if (invalidMap[index-1] < 0) rhs[i] += depths[index-1];
        else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index-1],-1));
      }
      if (!right) {
        if (invalidMap[index+1] < 0) rhs[i] += depths[index+1];
        else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index+1],-1));
      }
      if (!top) {
        if (invalidMap[index-w] < 0) rhs[i] += depths[index-w];
        else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index-w],-1));
      }
      if (!bottom) {
        if (invalidMap[index+w] < 0) rhs[i] += depths[index+w];
        else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index+w],-1));
      }
    }

    // solve sparse system
    Eigen::SparseMatrix<float> lhs(invalidList.size(), invalidList.size());
    lhs.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SimplicialLLT<Eigen::SparseMatrix<float> > solver;
    solver.compute(lhs);
    if (solver.info() == Eigen::Success) {
      Eigen::VectorXf solution = solver.solve(rhs);

      // fill in values
      for (size_t i = 0; i < invalidList.size(); ++i) {
        if (pixelsToUpdate[invalidList[i]]) {
          depths[invalidList[i]] = solution[i];
        }
      }
      img->setData(depths, type);
    }
    else {
      std::cout << "Error: cannot solve sparse system" << std::endl;
    }
  }

  void fillViewPlanar(maps::DepthImageView::Ptr& iView) {
    maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
    maps::DepthImage::Ptr img = iView->getDepthImage();
    std::vector<float>& depths =
      const_cast<std::vector<float>&>(img->getData(type));

    // gather valid points
    std::vector<Eigen::Vector3f> points;
    points.reserve(depths.size());
    const float invalidValue = img->getInvalidValue(type);
    int w(img->getWidth()), h(img->getHeight());
    for (int i = 0, idx = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j, ++idx) {
        float z = depths[idx];
        if (z == invalidValue) continue;
        points.push_back(Eigen::Vector3f(j,i,z));
      }
    }
    int n = points.size();

    // set up equations
    Eigen::VectorXf rhs(n);
    Eigen::MatrixXf lhs(n,3);
    for (int i = 0; i < n; ++i) {
      rhs[i] = -points[i][2];
      lhs(i,0) = points[i][0];
      lhs(i,1) = points[i][1];
      lhs(i,2) = 1;
    }

    // plane fit
    Eigen::Vector3f sol;
    Eigen::VectorXf weights = Eigen::VectorXf::Ones(n);
    Eigen::VectorXf weightsPrev = weights;
    const float sigma2 = 0.1*0.1;
    const float weightThresh = 1e-3f * 1e-3f * n;
    for (int iter = 0; iter < 10; ++iter) {
      // solve
      Eigen::MatrixXf a = weights.asDiagonal()*lhs;
      Eigen::VectorXf b = weights.asDiagonal()*rhs;
      sol = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(b);

      // re-compute weights
      Eigen::VectorXf e2 = lhs*sol - rhs;
      e2 = e2.array()*e2.array();
      weights = 1-e2.array()/sigma2;
      weights = (weights.array() < 0).select(0.0f, weights);

      // check for convergence
      Eigen::VectorXf weightDiff = weightsPrev-weights;
      if (weightDiff.dot(weightDiff) < weightThresh) break;
      weightsPrev = weights;
    }

    // fill in values
    for (int i = 0, idx = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j, ++idx) {
        float z = depths[idx];
        if (z != invalidValue) continue;
        depths[idx] = -(sol[0]*j + sol[1]*i + sol[2]);
      }
    }
    img->setData(depths, type);
  }

  maps::DepthImageView::Ptr getView() {
    // TODO: make this check how much time has elapsed since last receipt
    mViewClientWrapper->requestHeightMap();

    auto view = mViewClientWrapper->getView();
    if (view != NULL) {
      view->setNormalMethod(maps::DepthImageView::NormalMethodLeastSquares);
      view->setNormalRadius(mNormalRadius);
      if (mShouldFill && (mLastUpdateTime != view->getUpdateTime())) {
        //fillView(view);
        fillViewPlanar(view);
        mLastUpdateTime = view->getUpdateTime();
      }
    }
    return view;
  }

};

struct StateCollection {
  typedef std::unordered_map<int,std::shared_ptr<State> > StateGroup;
  StateGroup mStates;
  int mNextId;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<LcmLoop> mLcmLoop;

  StateCollection() {
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    drc::Clock::instance()->setVerbose(false);
    mLcmLoop.reset(new LcmLoop(mLcm));
    mNextId = 1;
  }

  ~StateCollection() {
    cleanupAll();

    // TODO: don't know why, but segfaults on exit without this line
    //   crashes in clock destructor, internal reference count of
    //   shared pointer to lcm object is invalid memory (from valgrind)
    // valgrind shows no errors when this line is present
    drc::Clock::instance()->setLcm(std::shared_ptr<lcm::LCM>());
  }

  void cleanupAll() {
    mLcmLoop->stop();
    if (mStates.size() > 0) {
      mexPrintf("destroying all map instances\n"); mexEvalString("drawnow");
      mexPrintf("WARNING: all handles now invalid\n"); mexEvalString("drawnow");
    }
    mStates.clear();
    //if (mexIsLocked()) mexUnlock();
  }

  std::shared_ptr<State> createState() {
    std::shared_ptr<State> state(new State(mNextId, mLcm));
    ++mNextId;
    mStates[state->mId] = state;
    // TODO TEMP? if (!mexIsLocked()) mexLock();
    mLcmLoop->start();
    return state;
  }

  std::shared_ptr<State> getState(const int iId) {
    StateGroup::const_iterator item = mStates.find(iId);
    if (item == mStates.end()) return std::shared_ptr<State>();
    return item->second;
  }

  void destroyState(const int iId) {
    mStates.erase(iId);
    if (mStates.size() == 0) {
      //if (mexIsLocked()) mexUnlock();
      mLcmLoop->stop();
    }
  }

  static StateCollection& instance() {
    static StateCollection theCollection;
    return theCollection;
  }

};

struct Utils {
  static std::string getString(const mxArray* iArray) {
    char* chars = mxArrayToString(iArray);
    std::string str = chars;
    mxFree(chars);
    return str;
  }

  static bool getBool(const std::string& iString) {
    std::string str = iString;
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    if ((str == "yes") || (str == "1") || (str == "on") ||
        (str == "true")) {
      return true;
    }
    else if ((str == "no") || (str == "0") || (str == "off") ||
             (str == "false")) {
      return false;
    }
    else {
      mexErrMsgTxt("MapWrapper: value must be true or false");
    }
  }
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  // first determine which command to apply
  std::string command;
  if (nrhs == 0) command = "create";
  else {
    if (!mxIsChar(prhs[0])) {
      mexErrMsgTxt("MapWrapper: first argument must be command string");
    }
    command = Utils::getString(prhs[0]);
  }
  std::transform(command.begin(), command.end(), command.begin(), ::tolower);

  // create instance
  if (command == "create") {
    auto state = StateCollection::instance().createState();
    if (nrhs > 1) {
      if ((nrhs % 2) != 1) {
        mexErrMsgTxt("MapWrapper: wrong number of parameter args");
      }
      for (int i = 1; i < nrhs; i += 2) {
        if (!mxIsChar(prhs[i])) {
          mexErrMsgTxt("MapWrapper: key must be string");
        }
        if (!mxIsChar(prhs[i+1])) {
          mexErrMsgTxt("MapWrapper: val must be string");
        }
        std::string key = Utils::getString(prhs[i]);
        std::string value = Utils::getString(prhs[i+1]);

        std::transform(key.begin(), key.end(), key.begin(), ::tolower);
        if (key == "privatechannel") {
          bool isPrivate = Utils::getBool(value);
          if (isPrivate) {
            state->mViewClientWrapper->
              setHeightMapChannel(kHeightMapChannel, kHeightMapViewId);
          }
          else {
            state->mViewClientWrapper->setHeightMapChannel
              ("MAP_DEPTH", drc::data_request_t::HEIGHT_MAP_SCENE);
          }
        }
        else {
          mexErrMsgTxt("MapWrapper: invalid property");
        }
      }  
    }

    state->start();

    // return index of state object
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double id = state->mId;
    memcpy(mxGetData(plhs[0]),&id,sizeof(double));
    return;
  }

  // get state object from static collection
  if ((nrhs < 2) || (mxGetNumberOfElements(prhs[1]) != 1)) {
    mexErrMsgTxt("MapWrapper: second argument must be handle id");
  }
  double val;
  memcpy(&val, mxGetData(prhs[1]), sizeof(double));
  int stateId = (int)(val+0.5);
  auto state = StateCollection::instance().getState(stateId);
  if (state == NULL) {
    mexErrMsgTxt("MapWrapper: handle is invalid; did you clear mex?");
  }

  // destroy instance
  if (command == "destroy") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to destroy");
    }
    StateCollection::instance().destroyState(state->mId);
  }

  // get point cloud
  else if (command == "pointcloud") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to pointcloud");
    }
    auto view = state->getView();
    if (view == NULL) {
      plhs[0] = mxCreateDoubleMatrix(3,0,mxREAL);
      return;
    }
    auto cloud = view->getAsPointCloud();
    plhs[0] = mxCreateDoubleMatrix(3,cloud->size(),mxREAL);
    double* ptr = mxGetPr(plhs[0]);
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType& pt = cloud->points[i];
      ptr[i*3+0] = pt.x;
      ptr[i*3+1] = pt.y;
      ptr[i*3+2] = pt.z;
    }
  }

  // get closest points and normals
  else if (command == "closest") {
    if (nrhs != 3) {
      mexErrMsgTxt("MapWrapper: too many arguments to closest");
    }
    if (mxGetM(prhs[2]) != 3) {
      mexErrMsgTxt("MapWrapper: points must be 3xn");
    }
    if (nlhs > 2) {
      mexErrMsgTxt("MapWrapper: too many output arguments");
    }

    int n = mxGetN(prhs[2]);
    plhs[0] = mxCreateDoubleMatrix(3,n,mxREAL);
    mxArray* pmxNormal = mxCreateDoubleMatrix(3,n,mxREAL);
    
    double* posPtr = mxGetPr(prhs[2]);
    double* closestPtr = mxGetPr(plhs[0]);
    double* normalPtr = mxGetPr(pmxNormal);
    Eigen::Vector3f pos, closest, normal;

    auto view = state->getView();
    for (int i = 0; i < n; ++i) {
      pos << posPtr[3*i], posPtr[3*i+1], posPtr[3*i+2];
      if ((view != NULL) && view->getClosest(pos,closest,normal)) {
        for (int j = 0; j < 3; ++j) {
          closestPtr[3*i+j] = closest[j];
          normalPtr[3*i+j] = normal[j];
        }
      }
      else { // invalid view or off grid
        for (int j = 0; j < 3; ++j) {
          closestPtr[3*i+j] = NAN;
          normalPtr[3*i+j] = NAN;
        }
      }
    }

    if (nlhs == 2) plhs[1] = pmxNormal;
    else mxDestroyArray(pmxNormal);
  }

  else if (command == "getrawdepth") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to getrawdepth");
    }
    auto view = state->getView();
    if (view == NULL) {
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
    auto img = view->getDepthImage();
    int w(img->getWidth()), h(img->getHeight());
    plhs[0] = mxCreateDoubleMatrix(h,w,mxREAL);
    auto vals = img->getData(maps::DepthImage::TypeDepth);
    float* inPtr = vals.data();
    double* outPtr = mxGetPr(plhs[0]);
    for (int i = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j, ++inPtr) {
        outPtr[j*h+i] = *inPtr;
      }
    }
  }

  else if (command == "setrawdepth") {
    if (nrhs != 3) {
      mexErrMsgTxt("MapWrapper: too many arguments to setrawdepth");
    }
    auto view = state->getView();
    if (view == NULL) {
      mexErrMsgTxt("MapWrapper: no view object, so not setting depths\n");
    }
    auto img = view->getDepthImage();
    int w(img->getWidth()), h(img->getHeight());
    if ((mxGetM(prhs[2]) != h) || (mxGetN(prhs[2]) != w)) {
      mexErrMsgTxt("MapWrapper: depth image sizes do not match");
    }

    std::vector<float> data(w*h);
    double* inPtr = mxGetPr(prhs[2]);
    for (int j = 0; j < w; ++j) {
      for (int i = 0; i < h; ++i, ++inPtr) {
        data[i*w+j] = (float)(*inPtr);
      }
    }
    img->setData(data, maps::DepthImage::TypeDepth);
  }

  else if (command == "transform") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to setrawdepth");
    }
    auto view = state->getView();
    plhs[0] = mxCreateDoubleMatrix(4,4,mxREAL);
    double* matx = mxGetPr(plhs[0]);
    if (view == NULL) {
      for (int i = 0; i < 16; ++i) matx[i] = 0;
      return;
    }
    Eigen::Projective3f transform = view->getTransform();
    for (int i = 0; i < 16; ++i) matx[i] = transform.data()[i];
  }

  else if (command == "property") {
    if (nrhs != 4) {
      mexErrMsgTxt("MapWrapper: must supply key/value pair");
    }
    if (!mxIsChar(prhs[2])) {
      mexErrMsgTxt("MapWrapper: property key must be string");
    }
    std::string key = Utils::getString(prhs[2]);
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);

    if (!mxIsChar(prhs[3])) {
      mexErrMsgTxt("MapWrapper: property value must be string");
    }
    std::string value = Utils::getString(prhs[3]);

    if (key == "normalradius") {
      std::istringstream(value) >> state->mNormalRadius;
    }
    else if (key == "fill") {
      state->mShouldFill = Utils::getBool(value);
    }
    else {
      mexErrMsgTxt("MapWrapper: invalid property");
    }
  }

  // none of the above
  else {
    mexErrMsgTxt("MapWrapper: invalid command");
  }
  
}
