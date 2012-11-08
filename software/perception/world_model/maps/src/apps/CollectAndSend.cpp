#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <pcl/common/transforms.h>

#include <lcmtypes/drc/heightmap_t.hpp>
#include <bot_core/timestamp.h>

using namespace std;

class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;

  State() {
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mManager.reset(new MapManager());
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

      // publish as octomap
      std::cout << "Publishing octomap..." << std::endl;
      octomap::raw_t raw = mState->mManager->getActiveMap()->getAsRaw();
      mState->mLcm->publish("OCTOMAP", &raw);

      // write pgm height map
      LocalMap::HeightMap heightMap =
        mState->mManager->getActiveMap()->getAsHeightMap();

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

      std::ofstream ofs("/home/mfallon/heightmap.txt");
      for (int i = 0; i < heightMap.mHeight; ++i) {
        for (int j = 0; j < heightMap.mWidth; ++j) {
          ofs << heightMap.mData[i*heightMap.mWidth+j] << " ";
        }
        ofs << std::endl;
      }
      std::cout << "Writing height map..." << std::endl;
    }
  }

protected:
  State* mState;
};

int main(const int iArgc, const char** iArgv) {
  State state;

  state.mLcm.reset(new lcm::LCM());
  if (!state.mLcm->good()) {
    cerr << "Cannot create lcm instance." << endl;
    return -1;
  }

  // TODO: temporary; need server
  BotParam* theParam = bot_param_new_from_file("/home/mfallon/drc/software/config/electic_deprecated/drc_robot.cfg");

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

  return 0;
}
