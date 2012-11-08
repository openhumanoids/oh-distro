#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>

#include <lcmtypes/drc/local_map_t.hpp>
#include <lcmtypes/drc/heightmap_t.hpp>
#include <bot_core/timestamp.h>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <pcl/common/transforms.h>

#include <bot_lcmgl_client/lcmgl.h>

using namespace std;

class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  bot_lcmgl_t* mLcmGl;

  State() {
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mManager.reset(new MapManager());
    mLcm.reset(new lcm::LCM());
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "map-debug");
  }

  ~State() {
    bot_lcmgl_destroy(mLcmGl);
  }
};

class DataConsumer {
public:
  DataConsumer(State* iState) {
    mState = iState;
    mCounter = 0;
  }

  void operator()() {
    while(true) {
      SensorDataReceiver::PointCloudWithPose data;
      if (mState->mSensorDataReceiver->waitForData(data)) {
        mState->mManager->addToBuffer(data.mTimestamp, data.mPointCloud,
                                      data.mPose);
        mState->mManager->getActiveMap()->add(data.mPointCloud, data.mPose);
      }
    }
  }

protected:
  State* mState;
  int mCounter;
};

class DataPublisher {
public:
  DataPublisher(State* iState) {
    mState = iState;
  }
  void operator()() {
    while(true) {
      // wait for timer expiry
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      timer.expires_from_now(boost::posix_time::milliseconds(3000));
      timer.wait();

      // see if map exists
      boost::shared_ptr<LocalMap> localMap = mState->mManager->getActiveMap();
      if (localMap == NULL) {
        continue;
      }

      // publish as local map
      std::vector<char> bytes;
      localMap->serialize(bytes);
      drc::local_map_t mapMessage;
      mapMessage.utime = bot_timestamp_now();
      mapMessage.id = localMap->getId();
      mapMessage.state_id = localMap->getStateId();
      mapMessage.size_bytes = bytes.size();
      mapMessage.data.insert(mapMessage.data.end(), bytes.begin(), bytes.end());
      mState->mLcm->publish("LOCAL_MAP", &mapMessage);
      cout << "Published local map (" << bytes.size() << " bytes)" << endl;


      // publish as octomap
      std::cout << "Publishing octomap..." << std::endl;
      octomap::raw_t raw = mState->mManager->getActiveMap()->getAsRaw();
      mState->mLcm->publish("OCTOMAP", &raw);

      // publish as height map
      std::cout << "Publishing height map..." << std::endl;
      LocalMap::HeightMap heightMap = localMap->getAsHeightMap();
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
      mState->mLcm->publish("HEIGHT_MAP", &heightMapMsg);

      if (true) {
        bot_lcmgl_t* lcmgl = mState->mLcmGl;
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

#define DrawTriangle_(a,b,c)\
            bot_lcmgl_begin(lcmgl, LCMGL_LINE_LOOP);\
            bot_lcmgl_vertex3f(lcmgl, a[0], a[1], a[2]);\
            bot_lcmgl_vertex3f(lcmgl, b[0], b[1], b[2]);\
            bot_lcmgl_vertex3f(lcmgl, c[0], c[1], c[2]);\
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
          }
        }
        bot_lcmgl_switch_buffer(lcmgl);
      }

    }
  }

protected:
  State* mState;
};

int main(const int iArgc, const char** iArgv) {
  State state;

  if (!state.mLcm->good()) {
    cerr << "Cannot create lcm instance." << endl;
    return -1;
  }

  BotParam* theParam =
    bot_param_new_from_server(state.mLcm->getUnderlyingLCM(), 0);

  state.mSensorDataReceiver->setLcm(state.mLcm);
  state.mSensorDataReceiver->setBotParam(theParam);
  state.mSensorDataReceiver->setMaxBufferSize(100);
  // TODO: temporary; make configurable
  state.mSensorDataReceiver->
    addChannel("ROTATING_SCAN",
               SensorDataReceiver::SensorTypePlanarLidar,
               "ROTATING_SCAN", "local");
  state.mSensorDataReceiver->start();

  state.mManager->setMapResolution(0.05);
  state.mManager->setMapDimensions(Eigen::Vector3d(10,10,10));
  state.mManager->setDataBufferLength(1000);
  state.mManager->createMap(Eigen::Isometry3d::Identity());

  DataConsumer consumer(&state);
  boost::thread thread(consumer);

  DataPublisher publisher(&state);
  boost::thread publishThread(publisher);
                                       
  while (0 == state.mLcm->handle());

  thread.join();
  publishThread.join();

  return 0;
}
