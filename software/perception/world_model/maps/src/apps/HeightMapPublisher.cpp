#include <maps/MapWrapper.hpp>
#include <maps/LocalMap.hpp>

#include <lcmtypes/drc/local_map_t.hpp>
#include <lcmtypes/drc/heightmap_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/heightmap_params_t.hpp>

#include <drc_utils/Clock.hpp>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>
#include <zlib.h>

#include <bot_lcmgl_client/lcmgl.h>

#include <ConciseArgs>

#include <iostream>

using namespace std;

class State : public MapWrapper::UpdateListener {
public:
  boost::shared_ptr<lcm::LCM> mLcm;
  bot_lcmgl_t* mLcmGl;
  lcm::Subscription* mParamsSubscription;
  MapWrapper mWrapper;
  std::string mHeightMapChannel;
  double mHeightMapResolution;
  float mMaxHeight;
  bool mShouldPublishLcmGl;
  bool mShouldCompress;
  bool mShouldUseFloat;
  
public:
  State() {
    mLcm.reset(new lcm::LCM());
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "map-debug");
    mParamsSubscription =
      mLcm->subscribe("HEIGHTMAP_PARAMS", &State::onHeightmapParams, this);
    mWrapper.setLcm(mLcm);
    drc::Clock::instance()->setLcm(mLcm);
    boost::shared_ptr<MapWrapper::UpdateListener> thisPtr(this);
    mWrapper.addListener(thisPtr);

    // defaults; these should be configured by command line args in main
    mWrapper.setMapChannel("LOCAL_MAP");
    mHeightMapChannel = "HEIGHT_MAP";
    mHeightMapResolution = 0.08;
    mMaxHeight = 1e10;
    mShouldPublishLcmGl = true;
    mShouldCompress = false;
    mShouldUseFloat = true;
  }

  ~State() {
    mWrapper.stop();
    bot_lcmgl_destroy(mLcmGl);
    mLcm->unsubscribe(mParamsSubscription);
  }

  void onHeightmapParams(const lcm::ReceiveBuffer* iBuf,
                         const std::string& iChannel,
                         const drc::heightmap_params_t* iMessage) {
    mHeightMapResolution = iMessage->resolution;
  }

  void notify(MapWrapper& iWrapper) {
    iWrapper.lock();
    MapWrapper::LocalMapConstPtr localMap = iWrapper.getMap();
    double factor = mHeightMapResolution/localMap->getResolution();
    int downSample = (int)floor(log(factor)/log(2) + 0.5);
    LocalMap::HeightMap heightMap =
      iWrapper.getMap()->getAsHeightMap(downSample, mMaxHeight);
    iWrapper.unlock();

    // publish legacy heightmap via lcm
    // TODO: this is deprecated
    cout << "Publishing legacy height map (downsample=" << downSample << ")...";
    drc::heightmap_t heightMapMsg;
    heightMapMsg.utime = drc::Clock::instance()->getCurrentTime();
    cout << "TODO " << heightMapMsg.utime << endl;
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
    mLcm->publish(mHeightMapChannel + "_DEPRECATED", &heightMapMsg);
    cout << "done." << endl;



    //
    // publish new heightmap via lcm
    //

    cout << "Publishing height map..." << endl;
    drc::map_image_t msg;

    // basic parameters
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.width = heightMap.mWidth;
    msg.height = heightMap.mHeight;
    msg.channels = 1;
    int sampleBytes = (mShouldUseFloat ? sizeof(float) : sizeof(uint8_t));
    msg.row_bytes = msg.width*sampleBytes;
    msg.total_bytes = msg.height*msg.row_bytes;
    msg.format = (mShouldUseFloat ? drc::map_image_t::FORMAT_GRAY_FLOAT32 :
                  drc::map_image_t::FORMAT_GRAY_UINT8);
    msg.compression = (mShouldCompress ? drc::map_image_t::COMPRESSION_ZLIB :
                       drc::map_image_t::COMPRESSION_NONE);

    // raw data conversion
    std::vector<uint8_t> bytes(msg.total_bytes);
    float scale(1), offset(0);
    if (mShouldUseFloat) {
      memcpy(&bytes[0], &heightMap.mData[0], msg.total_bytes);
    }
    else {
      std::fill(bytes.begin(), bytes.end(), 0);
      for (int i = 0; i < msg.width*msg.height; ++i) {
        if (heightMap.mData[i] < -1e10) {
          continue;
        }
        scale = 254. / (heightMap.mMaxZ - heightMap.mMinZ);
        offset = 1 - scale*heightMap.mMinZ;
        bytes[i] = floor(heightMap.mData[i]*scale + offset + 0.5);
      }
    }

    // transform from world to image
    Eigen::Affine3d xform = heightMap.mTransformToLocal.inverse();
    Eigen::Affine3d adjustment = Eigen::Affine3d::Identity();
    adjustment(2,2) = scale;
    adjustment(2,3) = offset;
    xform = adjustment.inverse() * xform;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        msg.transform[i][j] = xform(i,j);
      }
    }

    // compression
    if (mShouldCompress) {
      std::vector<uint8_t> compressedBytes(bytes.size()*1.001 + 12);
      unsigned long compressedSize = compressedBytes.size();
      compress2(&compressedBytes[0], &compressedSize,
                (const Bytef*)(&bytes[0]), bytes.size(),
                Z_BEST_SPEED);
      msg.total_bytes = (int)compressedSize;
      msg.data = compressedBytes;
    }
    else {
      msg.data = bytes;
    }

    // publish
    mLcm->publish(mHeightMapChannel, &msg);


    // debug: publish lcmgl to visualize height map
    if (mShouldPublishLcmGl) {
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
  opt.add(state.mShouldCompress, "z", "compress",
          "whether to compress height data");
  opt.add(state.mShouldUseFloat, "f", "float",
          "whether to use floating point data");
  opt.add(state.mShouldPublishLcmGl, "g", "lcmgl",
          "whether to publish lcmgl messages");
  opt.parse();
  state.mWrapper.setMapChannel(mapChannel);

  // start running
  state.mWrapper.start();

  // handle lcm messages
  while (0 == state.mLcm->handle());

  return 0;
}
