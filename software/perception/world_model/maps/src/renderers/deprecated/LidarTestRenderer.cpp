#include <iostream>
#include <unordered_map>
#include <thread>

#include <boost/circular_buffer.hpp>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <gtkmm.h>
#include <GL/gl.h>

#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>
#include <gtkmm-renderer/RendererBase.hpp>

#include <maps/Collector.hpp>
#include <maps/Surfelizer.hpp>
#include <maps/Utils.hpp>
#include <maps/BotWrapper.hpp>
#include "MeshRenderer.hpp"

using namespace maps;

struct LidarData {
  std::string mName;
  std::string mChannel;
  bool mActive;
  Eigen::Vector3f mColor;
  typedef std::shared_ptr<PointSet> ScanPtr;
  typedef boost::circular_buffer<ScanPtr> ScanBuffer;
  ScanBuffer mScans;
  Surfelizer mSurfelizer;
  typedef boost::circular_buffer<std::vector<Surfelizer::Surfel> > SurfelList;
  SurfelList mSurfels;

  typedef std::shared_ptr<LidarData> Ptr;

  LidarData() : mActive(false), mColor(0,0,0) {}
  ~LidarData() {}
};

class LidarTestRenderer :
  public gtkmm::RendererBase, public Collector::DataListener {
protected:
  enum DrawMode {
    DrawModePoints,
    DrawModeSurfelsWire,
    DrawModeSurfelsFilled
  };

  struct SurfelParams {
    int mSurfelMode;
    int mDecimation;
    bool operator==(const SurfelParams& iParams) {
      return ((iParams.mSurfelMode == mSurfelMode) &&
              (iParams.mDecimation == mDecimation));
    }
    bool operator!=(const SurfelParams& iParams) {
      return !(*this==iParams);
    }
  };

  typedef std::unordered_map<std::string, LidarData::Ptr> LidarDataMap;
  LidarDataMap mDataMap;
  maps::Collector mCollector;
  int mColorMode;
  SurfelParams mParams;
  int mDrawMode;
  SurfelParams mSurfelParams;
  SurfelParams mSurfelParamsPrev;
  double mPointSize;
  double mSurfelScale;
  double mMinZ;
  double mMaxZ;
  int mScanHistory;
  std::shared_ptr<maps::MeshRenderer> mMeshRenderer;
  std::vector<Eigen::Vector3f> mPointBuffer;
  std::mutex mSurfelMutex;
  
public:

  LidarTestRenderer(BotViewer* iViewer, const int iPriority,
                    const lcm_t* iLcm,
                    const BotParam* iParam, const BotFrames* iFrames)
    : gtkmm::RendererBase("Lidar Test", iViewer, iPriority, iLcm,
                          iParam, iFrames) {

    // set up ui widgets
    {
      std::vector<int> ids = {
        maps::MeshRenderer::ColorModeFlat,
        maps::MeshRenderer::ColorModeHeight,
        maps::MeshRenderer::ColorModeRange,
        maps::MeshRenderer::ColorModeCamera };
      std::vector<std::string> labels = { "Flat", "Height", "Range", "Camera"};
      mColorMode = maps::MeshRenderer::ColorModeHeight;
      addCombo("Color Mode", mColorMode, labels, ids);
    }

    {
      std::vector<int> ids = { DrawModePoints, DrawModeSurfelsWire,
                               DrawModeSurfelsFilled };
      std::vector<std::string> labels = { "Points", "Surfels Wire",
                                          "Surfels Filled"};
      mDrawMode = DrawModePoints;
      addCombo("Draw Mode", mDrawMode, labels, ids);
    }

    {
      std::vector<int> ids = {
        Surfelizer::SizeMethodFixed, Surfelizer::SizeMethodRange,
        Surfelizer::SizeMethodAngles, Surfelizer::SizeMethodNeighbors };
      std::vector<std::string> labels = {"Fixed", "Range", "Angles",
                                         "Neighbors"};
      mSurfelParams.mSurfelMode = Surfelizer::SizeMethodFixed;
      addCombo("Surfel Size", mSurfelParams.mSurfelMode, labels, ids);
    }

    mPointSize = 3;
    addSlider("Point Size", mPointSize, 0.1, 10, 0.1);

    mSurfelScale = 1;
    addSlider("Surfel Scale", mSurfelScale, 0.1, 2.0, 0.1);

    mMinZ = 0;
    addSlider("Z Scale Min", mMinZ, -1, 2, 0.01);

    mMaxZ = 1;
    addSlider("Z Scale Max", mMaxZ, -1, 2, 0.01);

    mScanHistory = 1000;
    addSpin("Scan History", mScanHistory, 0, 1000, 1);

    mSurfelParams.mDecimation = 1;
    addSlider("Decimation", mSurfelParams.mDecimation, 1, 100, 1);

    // set up internal variables
    mSurfelParamsPrev.mSurfelMode = -1;
    BotWrapper::Ptr botWrapper(new BotWrapper(getLcm(), getBotParam(),
                                              getBotFrames()));
    mMeshRenderer.reset(new MeshRenderer());
    mMeshRenderer->setBotObjects(getLcm(), getBotParam(), getBotFrames());
    mMeshRenderer->setCameraChannel("CAMERALEFT");
    mCollector.setBotWrapper(botWrapper);
    mCollector.addListener(*this);

    // subscribe to lcm channels and set up data structures
    BotParam* botParam = getBotParam();
    char** lidarNames = bot_param_get_all_planar_lidar_names(botParam);
    if (lidarNames != NULL) {
      for (int i = 0; lidarNames[i] != NULL; ++i) {
        char prefix[1024];
        bot_param_get_planar_lidar_prefix(botParam, lidarNames[i], prefix,
                                          sizeof(prefix));
        std::string key = std::string(prefix) + ".lcm_channel";
        char* str;
        bot_param_get_str(botParam, key.c_str(), &str);
        std::string channel(str);
        free(str);
        mCollector.getDataReceiver()->
          addChannel(channel, SensorDataReceiver::SensorTypePlanarLidar,
                     channel, "local");

        key = std::string(prefix) + ".viewer_color";
        double color[3];
        bot_param_get_double_array(botParam, key.c_str(), color, 3);

        LidarData::Ptr data(new LidarData());
        data->mName = lidarNames[i];
        data->mChannel = channel;
        data->mActive = false;
        data->mColor = Eigen::Vector3f(color[0], color[1], color[2]);;
        data->mScans.set_capacity(1000);
        data->mSurfels.set_capacity(1000);
        mDataMap[data->mChannel] = data;
      }
      g_strfreev(lidarNames);
    }

    mCollector.start();
  }

  ~LidarTestRenderer() {
  }

  void notify(const SensorDataReceiver::SensorData& iData) {
    LidarDataMap::const_iterator item = mDataMap.find(iData.mChannel);
    if (item == mDataMap.end()) return;
    {
      std::lock_guard<std::mutex> lock(mSurfelMutex);
      item->second->mScans.push_back(iData.mPointSet);
      item->second->mSurfels.
        push_back(item->second->mSurfelizer.addScan(*iData.mPointSet));
    }
    requestDraw();
  }

  void draw() {
    std::lock_guard<std::mutex> lock(mSurfelMutex);

    mPointBuffer.clear();

    // iterate over different laser sensors
    LidarDataMap::const_iterator dataIter = mDataMap.begin();
    for (; dataIter != mDataMap.end(); ++dataIter) {
      LidarData::Ptr data = dataIter->second;

      // set up mesh renderer parameters
      mMeshRenderer->setColor(data->mColor[0], data->mColor[1],
                              data->mColor[2]);
      mMeshRenderer->setPointSize(mPointSize);
      mMeshRenderer->setColorMode((maps::MeshRenderer::ColorMode)mColorMode);
      mMeshRenderer->setScaleRange(mMinZ, mMaxZ);
      BotTrans trans;
      bot_frames_get_trans(getBotFrames(), "head", "local", &trans);
      Eigen::Vector3f pos(trans.trans_vec[0], trans.trans_vec[1],
                          trans.trans_vec[2]);
      mMeshRenderer->setRangeOrigin(pos);

      // draw points only
      if (mDrawMode == DrawModePoints) {
        int counter = 0;
        LidarData::ScanBuffer::reverse_iterator iter = data->mScans.rbegin();
        for (; (iter != data->mScans.rend()) && (counter < mScanHistory);
             ++iter, ++counter) {
          LidarData::ScanPtr scan = *iter;
          Eigen::Isometry3f pose = Utils::getPose(*scan->mCloud);
          Eigen::Vector3f origin = pose.translation();

          // grab all valid points from this scan
          mPointBuffer.reserve(mPointBuffer.size() + scan->mCloud->size());
          for (size_t i = 0; i < scan->mCloud->size(); ++i) {
            // TODO: this is duplicate computation
            Eigen::Vector3f pt = pose * (*scan->mCloud)[i].getVector3fMap();
            float range = (pt - origin).norm();
            if ((range >= scan->mMinRange) && (range <= scan->mMaxRange)) {
              mPointBuffer.push_back(pt);
            }
          }
        }
        mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModePoints);
        mMeshRenderer->setData(mPointBuffer, std::vector<Eigen::Vector3i>());
      }

      // draw surfels (wireframe)
      else if ((mDrawMode == DrawModeSurfelsWire) ||
               (mDrawMode == DrawModeSurfelsFilled)) {
        std::vector<Eigen::Vector3f> vertices;
        std::vector<Eigen::Vector3i> faces;
        computeSurfels(data, vertices, faces);
        mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModeWireframe);
        if (mDrawMode == DrawModeSurfelsFilled) {
          mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModeFilled);
        }
        mMeshRenderer->setData(vertices, faces);
      }

      // execute gl drawing
      mMeshRenderer->draw();
    }
  }


  void computeSurfels(const LidarData::Ptr& iData,
                      std::vector<Eigen::Vector3f>& oVertices,
                      std::vector<Eigen::Vector3i>& oFaces) {

    if (iData->mScans.size() == 0) return;

    if (mSurfelParams != mSurfelParamsPrev) {
      // set up surfelizer
      iData->mSurfelizer.setScanRadius(2);
      iData->mSurfelizer.setPointRadius(2);
      iData->mSurfelizer.
        setSizeMethod((Surfelizer::SizeMethod)mSurfelParams.mSurfelMode);
      iData->mSurfelizer.setNominalSize(0.01);
      iData->mSurfelizer.setDecimation(mSurfelParams.mDecimation);
      iData->mSurfels.clear();

      // loop over appropriate scans
      const int numScans = iData->mScans.size();
      const int stopIndex = std::max(0, numScans - mScanHistory - 1);
      for (int i = numScans-1; i >= stopIndex; --i) {
        std::vector<Surfelizer::Surfel> surfels = 
          iData->mSurfelizer.addScan(*iData->mScans[i]);
        iData->mSurfels.push_back(surfels);
      }
      mSurfelParamsPrev = mSurfelParams;
    }

    // allocate vectors
    const int numScans = iData->mSurfels.size();
    const int stopIndex = std::max(0, numScans - mScanHistory - 1);
    int totalSurfels = 0;
    for (int i = numScans-1; i >= stopIndex; --i) {
      totalSurfels += iData->mSurfels[i].size();
    }
    oVertices.resize(totalSurfels*4);
    oFaces.resize(totalSurfels*2);

    // form quads
    int cnt = 0;
    for (int i = numScans-1; i >= stopIndex; --i) {
      for (size_t j = 0; j < iData->mSurfels[i].size(); ++j, ++cnt) {

        // scale principal vectors
        Surfelizer::Surfel& s = iData->mSurfels[i][j];
        Eigen::Vector3f v1 = s.mOrientation.col(0)*s.mSize[0]*mSurfelScale/2;
        Eigen::Vector3f v2 = s.mOrientation.col(1)*s.mSize[1]*mSurfelScale/2;

        // create a quad (two triangles) for this point
        oVertices[cnt*4+0] = s.mCenter -v1 +v2;
        oVertices[cnt*4+1] = s.mCenter +v1 +v2;
        oVertices[cnt*4+2] = s.mCenter +v1 -v2;
        oVertices[cnt*4+3] = s.mCenter -v1 -v2;
        oFaces[cnt*2+0] = Eigen::Vector3i(4*cnt+0,4*cnt+3,4*cnt+1);
        oFaces[cnt*2+1] = Eigen::Vector3i(4*cnt+1,4*cnt+3,4*cnt+2);
      }
    }
  }
};


// this is the single setup method exposed for integration with the viewer
void lidartest_renderer_setup(BotViewer* iViewer, const int iPriority,
                              const lcm_t* iLcm,
                              const BotParam* iParam,
                              const BotFrames* iFrames) {
  new LidarTestRenderer(iViewer, iPriority, iLcm, iParam, iFrames);
}
