#include <mex.h>

#include <ctime>
#include <memory>
#include <unordered_map>
#include <sstream>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

#include <lcmtypes/drc/data_request_t.hpp>

#include <terrain-map/TerrainMap.hpp>

using terrainmap::TerrainMap;

namespace mexmaps {

const std::string kHeightMapChannel = "MAP_CONTROL_HEIGHT";
const int kHeightMapViewId = 1000;

struct MapCollection {
  typedef std::unordered_map<int,std::shared_ptr<TerrainMap> > MapGroup;
  MapGroup mMaps;
  int mNextId;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<drc::BotWrapper> mBotWrapper;

  MapCollection() {
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    drc::Clock::instance()->setVerbose(false);
    mLcmWrapper.reset(new drc::LcmWrapper(mLcm));
    mBotWrapper.reset(new drc::BotWrapper(mLcm, NULL, NULL));
    mNextId = 1;
  }

  ~MapCollection() {
    cleanupAll();

    // TODO: don't know why, but segfaults on exit without this line
    //   crashes in clock destructor, internal reference count of
    //   shared pointer to lcm object is invalid memory (from valgrind)
    // valgrind shows no errors when this line is present
    drc::Clock::instance()->setLcm(std::shared_ptr<lcm::LCM>());
  }

  void cleanupAll() {
    mLcmWrapper->stopHandleThread();
    if (mMaps.size() > 0) {
      mexPrintf("destroying all map instances\n"); mexEvalString("drawnow");
      mexPrintf("WARNING: all handles now invalid\n"); mexEvalString("drawnow");
    }
    mMaps.clear();
  }

  std::shared_ptr<TerrainMap> createMap() {
    std::shared_ptr<TerrainMap> terrainMap(new TerrainMap(mBotWrapper));
    terrainMap->setInfo(kHeightMapViewId, kHeightMapChannel);
    mMaps[mNextId] = terrainMap;
    ++mNextId;
    mLcmWrapper->startHandleThread();
    return terrainMap;
  }

  std::shared_ptr<TerrainMap> getMap(const int iId) const {
    MapGroup::const_iterator item = mMaps.find(iId);
    if (item == mMaps.end()) return std::shared_ptr<TerrainMap>();
    return item->second;
  }

  void destroyMap(const int iId) {
    mMaps.erase(iId);
    if (mMaps.size() == 0) {
      mLcmWrapper->stopHandleThread();
    }
  }

  int getMapId(const std::shared_ptr<TerrainMap>& iMap) const {
    for (auto m : mMaps) {
      if (m.second == iMap) return m.first;
    }
    return -1;
  }

  static MapCollection& instance() {
    static MapCollection theCollection;
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
    return false;
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
    auto terrainMap = MapCollection::instance().createMap();
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
            terrainMap->setInfo(kHeightMapViewId, kHeightMapChannel);
          }
          else {
            terrainMap->setInfo(drc::data_request_t::HEIGHT_MAP_SCENE,
                                "MAP_DEPTH");
          }
        }
        else {
          mexErrMsgTxt("MapWrapper: invalid property");
        }
      }
      terrainMap->startListening();
      if (terrainMap->getViewId() == kHeightMapViewId) {
        terrainMap->sendRequest(Eigen::Vector3d(-2,-5,-3),
                                Eigen::Vector3d(5,5,0.3), 0.03, 5, 0.5);
      }
    }

    // return index of terrainmap object
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double id = MapCollection::instance().getMapId(terrainMap);
    memcpy(mxGetData(plhs[0]),&id,sizeof(double));
    return;
  }

  // get terrainmap object from static collection
  if ((nrhs < 2) || (mxGetNumberOfElements(prhs[1]) != 1)) {
    mexErrMsgTxt("MapWrapper: second argument must be handle id");
  }
  double val;
  memcpy(&val, mxGetData(prhs[1]), sizeof(double));
  int mapId = (int)(val+0.5);
  auto terrainMap = MapCollection::instance().getMap(mapId);
  if (terrainMap == NULL) {
    mexErrMsgTxt("MapWrapper: handle is invalid; did you clear mex?");
  }

  // destroy instance
  if (command == "destroy") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to destroy");
    }
    MapCollection::instance().destroyMap(mapId);
  }

  /* TODO: deprecated
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
  */

  // get heights and normals at specified xy positions
  else if (command == "terrain") {
    if (nrhs != 3) {
      mexErrMsgTxt("MapWrapper: too many arguments to terrain");
    }
    int m = mxGetM(prhs[2]);
    if (m < 2) {
      mexErrMsgTxt("MapWrapper: points must be at least 2xn");
    }
    if (nlhs > 2) {
      mexErrMsgTxt("MapWrapper: too many output arguments");
    }

    int n = mxGetN(prhs[2]);
    plhs[0] = mxCreateDoubleMatrix(1,n,mxREAL);
    mxArray* pmxNormal = mxCreateDoubleMatrix(3,n,mxREAL);
    
    double* posPtr = mxGetPr(prhs[2]);
    double* heightPtr = mxGetPr(plhs[0]);
    double* normalPtr = mxGetPr(pmxNormal);
    Eigen::Vector3d normal;

    for (int i = 0; i < n; ++i) {
      terrainMap->getHeightAndNormal(posPtr[m*i], posPtr[m*i+1],
                                     heightPtr[i], normal);
      for (int j = 0; j < 3; ++j) {
        normalPtr[3*i+j] = normal[j];
      }
    }

    if (nlhs == 2) plhs[1] = pmxNormal;
    else mxDestroyArray(pmxNormal);
  }

  else if (command == "heightdata") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to heightdata");
    }
    auto data = terrainMap->getData();
    if (data == NULL) {
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
    else {
      int w(data->mWidth), h(data->mHeight);
      plhs[0] = mxCreateDoubleMatrix(h,w,mxREAL);
      float* inPtr = data->mHeights.data();
      double* outPtr = mxGetPr(plhs[0]);
      for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j, ++inPtr) {
          outPtr[j*h+i] = *inPtr;
        }
      }
    }
    if (nlhs > 1) {
      auto transform = (data == NULL) ? Eigen::Projective3d::Identity() :
        data->mTransform;
      plhs[1] = mxCreateDoubleMatrix(4,4,mxREAL);
      double* outPtr = mxGetPr(plhs[1]);
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          outPtr[j*4+i] = transform(i,j);
        }
      }
    }
  }

  else if (command == "fillplane") {
    if (nlhs > 0) {
      mexErrMsgTxt("MapWrapper: too many output arguments to fillplane");
    }
    if (nrhs != 3) {
      mexErrMsgTxt("MapWrapper: need to specify fillplane");
    }
    if (mxGetNumberOfElements(prhs[2]) != 4) {
      mexErrMsgTxt("MapWrapper: fillplane must be 4-vector");
    }
    double* ptr = mxGetPr(prhs[2]);
    Eigen::Vector4d fillPlane(ptr[0], ptr[1], ptr[2], ptr[3]);
    terrainMap->setFillPlane(fillPlane);
  }

  /* TODO: deprecated
  else if (command == "setrawdepth") {
    if (nrhs != 3) {
      mexErrMsgTxt("MapWrapper: too many arguments to setrawdepth");
    }
    auto view = std::dynamic_pointer_cast<maps::DepthImageView>
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
      mexErrMsgTxt("MapWrapper: too many arguments to transform");
    }
    plhs[0] = mxCreateDoubleMatrix(4,4,mxREAL);
    double* matx = mxGetPr(plhs[0]);
    memset(matx, 16*sizeof(double), 0);
    auto data = terrainMap->getData();
    if (data != NULL) {
      for (int i = 0; i < 16; ++i) matx[i] = data->mTransform.data()[i];
    }
  }
  */

  else if (command == "pointer") {
    if (nrhs != 2) {
      mexErrMsgTxt("MapWrapper: too many arguments to pointer");
    }
    auto ptr = terrainMap.get();
    mxClassID classId = (sizeof(ptr)==4) ? mxUINT32_CLASS : mxUINT64_CLASS;
    plhs[0] = mxCreateNumericMatrix(1,1,classId,mxREAL);
    memcpy(mxGetPr(plhs[0]), &ptr, sizeof(ptr));
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

    if (key == "fillmissing") {
      terrainMap->shouldFillMissing(Utils::getBool(value));
    }
    else if (key == "overrideheights") {
      terrainMap->overrideHeights(Utils::getBool(value));
    }
    else if (key == "normalmethod") {
      int method;
      std::istringstream(value) >> method;
      terrainMap->setNormalMethod((TerrainMap::NormalMethod)method);
    }
    else if (key == "usefootpose") {
      terrainMap->useFootPose(Utils::getBool(value));
    }
    else if (key == "normalradius") {
      double normalRadius;
      std::istringstream(value) >> normalRadius;
      terrainMap->setNormalRadius(normalRadius);
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
