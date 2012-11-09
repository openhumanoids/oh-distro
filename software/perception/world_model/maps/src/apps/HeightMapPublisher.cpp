#include <maps/MapWrapper.hpp>
#include <maps/LocalMap.hpp>

#include <lcmtypes/drc/local_map_t.hpp>
#include <lcmtypes/drc/heightmap_t.hpp>
#include <bot_core/timestamp.h>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>

#include <bot_lcmgl_client/lcmgl.h>

#include <ConciseArgs>

#include <iostream>

using namespace std;

class State : public MapWrapper::UpdateListener {
public:
  boost::shared_ptr<lcm::LCM> mLcm;
  bot_lcmgl_t* mLcmGl;
  MapWrapper mWrapper;
  std::string mHeightMapChannel;
  double mHeightMapResolution;
  float mMaxHeight;
  bool mPublishLcmGl;
  
public:
  State() {
    mLcm.reset(new lcm::LCM());
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "map-debug");
    mWrapper.setLcm(mLcm);
    boost::shared_ptr<MapWrapper::UpdateListener> thisPtr(this);
    mWrapper.addListener(thisPtr);

    // defaults; these should be configured by command line args in main
    mWrapper.setMapChannel("LOCAL_MAP");
    mHeightMapChannel = "HEIGHT_MAP";
    mHeightMapResolution = 0.08;
    mMaxHeight = 1e10;
    mPublishLcmGl = true;
  }

  ~State() {
    mWrapper.stop();
    bot_lcmgl_destroy(mLcmGl);
  }

  void notify(MapWrapper& iWrapper) {
    iWrapper.lock();
    MapWrapper::LocalMapConstPtr localMap = iWrapper.getMap();
    double factor = mHeightMapResolution/localMap->getResolution();
    int downSample = log(factor)/log(2);
    LocalMap::HeightMap heightMap =
      iWrapper.getMap()->getAsHeightMap(downSample, mMaxHeight);
    iWrapper.unlock();

    // publish heightmap via lcm
    cout << "Publishing height map (downsample=" << downSample << ")...";
    drc::heightmap_t heightMapMsg;
    heightMapMsg.utime = bot_timestamp_now();
    heightMapMsg.nx = heightMap.mWidth;
    heightMapMsg.ny = heightMap.mHeight;
    heightMapMsg.npix = heightMapMsg.nx * heightMapMsg.ny;
    Eigen::Vector3d p0 = heightMap.mTransformToLocal*Eigen::Vector3d(0,0,0);
    Eigen::Vector3d px = heightMap.mTransformToLocal*Eigen::Vector3d(1,0,0);
    Eigen::Vector3d py = heightMap.mTransformToLocal*Eigen::Vector3d(0,1,0);
    heightMapMsg.scale_x = (px-p0).norm();
    heightMapMsg.scale_y = (py-p0).norm();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        heightMapMsg.transform_to_local[i][j] =
          heightMap.mTransformToLocal(i,j);
      }
    }
    heightMapMsg.heights = heightMap.mData;
    mLcm->publish(mHeightMapChannel, &heightMapMsg);
    cout << "done." << endl;

    // debug: publish lcmgl to visualize height map
    if (mPublishLcmGl) {
      cout << "Publishing lcmgl...";
      bot_lcmgl_t* lcmgl = mLcmGl;
      bot_lcmgl_color3f(lcmgl, 1.0f, 0.5f, 0.0f);
      for (int i = 0; i < heightMap.mHeight-1; ++i) {
        for (int j = 0; j < heightMap.mWidth-1; ++j) {
          int index = i*heightMap.mWidth + j;
          double z00 = heightMap.mData[index];
          double z10 = heightMap.mData[index+1];
          double z01 = heightMap.mData[index+heightMap.mWidth];
          double z11 = heightMap.mData[index+heightMap.mWidth+1];
          bool valid00 = z00 > -1e10;
          bool valid10 = z10 > -1e10;
          bool valid01 = z01 > -1e10;
          bool valid11 = z11 > -1e10;
          int validSum = (int)valid00 + (int)valid10 +
            (int)valid01 + (int)valid11;
          if (validSum < 3) {
            continue;
          }

          Eigen::Affine3f xform = heightMap.mTransformToLocal.cast<float>();
          Eigen::Vector3f p00 = xform*Eigen::Vector3f(j,i,z00);
          Eigen::Vector3f p10 = xform*Eigen::Vector3f(j+1,i,z10);
          Eigen::Vector3f p01 = xform*Eigen::Vector3f(j,i+1,z01);
          Eigen::Vector3f p11 = xform*Eigen::Vector3f(j+1,i+1,z11);

#define DrawTriangle_(a,b,c)                            \
          bot_lcmgl_begin(lcmgl, LCMGL_LINE_LOOP);      \
          bot_lcmgl_vertex3f(lcmgl, a[0], a[1], a[2]);  \
          bot_lcmgl_vertex3f(lcmgl, b[0], b[1], b[2]);  \
          bot_lcmgl_vertex3f(lcmgl, c[0], c[1], c[2]);  \
          bot_lcmgl_end(lcmgl);

          if (validSum == 4) {
            DrawTriangle_(p00, p10, p01);
            DrawTriangle_(p11, p10, p01);
          }

          else {
            if (!valid00) {
              DrawTriangle_(p10, p01, p11);
            }
            else if (!valid10) {
              DrawTriangle_(p00, p01, p11);
            }
            else if (!valid01) {
              DrawTriangle_(p00, p11, p10);
            }
            else if (!valid11) {
              DrawTriangle_(p00, p01, p10);
            }
          }
#undef DrawTriangle_
        }
      }
      bot_lcmgl_switch_buffer(lcmgl);
      cout << "done" << endl;
    }
  }
};

int main(const int iArgc, const char** iArgv) {
  // instantiate state
  State state;

  // parse arguments
  string mapChannel = "LOCAL_MAP";
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(mapChannel, "m", "map_channel",
          "channel to listen for local maps");
  opt.add(state.mHeightMapChannel, "c", "heightmap_channel",
          "channel to publish height maps");
  opt.add(state.mHeightMapResolution, "r", "resolution",
          "approximate desired heightmap resolution");
  opt.add(state.mMaxHeight, "x", "max_height",
          "maximum height to consider when creating height map");
  opt.add(state.mPublishLcmGl, "g", "lcmgl",
          "whether to publish lcmgl messages");
  opt.parse();
  state.mWrapper.setMapChannel(mapChannel);

  // start running
  state.mWrapper.start();

  // handle lcm messages
  while (0 == state.mLcm->handle());

  return 0;
}
