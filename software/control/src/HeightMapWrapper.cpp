#include <mex.h>

#include <ctime>
#include <memory>
#include <unordered_map>
#include <sstream>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>
#include <drc_utils/LcmWrapper.hpp>

#include <lcmtypes/drc/data_request_t.hpp>

#include <maps/DepthImageView.hpp>
#include <maps/DepthImage.hpp>

#include "mexmaps/MapLib.hpp"
#include "mexmaps/ViewClientWrapper.hpp"

namespace mexmaps {

struct WrapperCollection {
  typedef std::unordered_map<int,std::shared_ptr<ViewClientWrapper> > WrapperGroup;
  WrapperGroup mWrappers;
  int mNextId;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;

  WrapperCollection() {
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    drc::Clock::instance()->setVerbose(false);
    mLcmWrapper.reset(new drc::LcmWrapper(mLcm));
    mNextId = 1;
  }

  ~WrapperCollection() {
    cleanupAll();

    // TODO: don't know why, but segfaults on exit without this line
    //   crashes in clock destructor, internal reference count of
    //   shared pointer to lcm object is invalid memory (from valgrind)
    // valgrind shows no errors when this line is present
    drc::Clock::instance()->setLcm(std::shared_ptr<lcm::LCM>());
  }

  void cleanupAll() {
    mLcmWrapper->stopHandleThread();
    if (mWrappers.size() > 0) {
      mexPrintf("destroying all map instances\n"); mexEvalString("drawnow");
      mexPrintf("WARNING: all handles now invalid\n"); mexEvalString("drawnow");
    }
    mWrappers.clear();
    //if (mexIsLocked()) mexUnlock();
  }

  std::shared_ptr<ViewClientWrapper> createWrapper() {
    std::shared_ptr<ViewClientWrapper> wrapper(new ViewClientWrapper(mNextId, mLcm));
    ++mNextId;
    mWrappers[wrapper->mId] = wrapper;
    // TODO TEMP? if (!mexIsLocked()) mexLock();
    mLcmWrapper->startHandleThread();
    return wrapper;
  }

  std::shared_ptr<ViewClientWrapper> getWrapper(const int iId) {
    WrapperGroup::const_iterator item = mWrappers.find(iId);
    if (item == mWrappers.end()) return std::shared_ptr<ViewClientWrapper>();
    return item->second;
  }

  void destroyWrapper(const int iId) {
    mWrappers.erase(iId);
    if (mWrappers.size() == 0) {
      //if (mexIsLocked()) mexUnlock();
      mLcmWrapper->stopHandleThread();
    }
  }

  static WrapperCollection& instance() {
    static WrapperCollection theCollection;
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

}

using namespace mexmaps;

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
    auto wrapper = WrapperCollection::instance().createWrapper();
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
            wrapper->setHeightMapChannel(MapHandle::kHeightMapChannel,
                                         MapHandle::kHeightMapViewId);
          }
          else {
            wrapper->setHeightMapChannel
              ("MAP_DEPTH", drc::data_request_t::HEIGHT_MAP_SCENE);
          }
        }
        else {
          mexErrMsgTxt("MapWrapper: invalid property");
        }
      }  
    }

    wrapper->start();

    // return index of wrapper object
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double id = wrapper->mId;
    memcpy(mxGetData(plhs[0]),&id,sizeof(double));
    return;
  }

  // get wrapper object from static collection
  if ((nrhs < 2) || (mxGetNumberOfElements(prhs[1]) != 1)) {
    mexErrMsgTxt("MapWrapper: second argument must be handle id");
  }
  double val;
  memcpy(&val, mxGetData(prhs[1]), sizeof(double));
  int wrapperId = (int)(val+0.5);
  auto wrapper = WrapperCollection::instance().getWrapper(wrapperId);
  if (wrapper == NULL) {
    mexErrMsgTxt("MapWrapper: handle is invalid; did you clear mex?");
  }

  // destroy instance
  if (command == "destroy") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to destroy");
    }
    WrapperCollection::instance().destroyWrapper(wrapper->mId);
  }

  // get point cloud
  else if (command == "pointcloud") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to pointcloud");
    }
    auto view = wrapper->getView();
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

    auto view = wrapper->getView();
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
    auto view = std::static_pointer_cast<maps::DepthImageView>
      (wrapper->getView());
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
    auto view = std::static_pointer_cast<maps::DepthImageView>
      (wrapper->getView());
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
    auto view = wrapper->getView();
    plhs[0] = mxCreateDoubleMatrix(4,4,mxREAL);
    double* matx = mxGetPr(plhs[0]);
    if (view == NULL) {
      for (int i = 0; i < 16; ++i) matx[i] = 0;
      return;
    }
    Eigen::Projective3f transform = view->getTransform();
    for (int i = 0; i < 16; ++i) matx[i] = transform.data()[i];
  }

  else if (command == "wrapper") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to wrapper");
    }
    auto ptr = wrapper->mHandle.get();
    mxClassID classId = (sizeof(ptr)==4) ? mxUINT32_CLASS : mxUINT64_CLASS;
    plhs[0] = mxCreateNumericMatrix(1,1,classId,mxREAL);
    memcpy(mxGetPr(plhs[0]), ptr, sizeof(ptr));
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
      std::istringstream(value) >> wrapper->mNormalRadius;
    }
    else if (key == "fill") {
      wrapper->mShouldFill = Utils::getBool(value);
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
