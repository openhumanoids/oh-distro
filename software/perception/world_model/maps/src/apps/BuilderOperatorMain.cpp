#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/DeltaReceiver.hpp>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/heightmap_t.hpp>
#include <lcmtypes/drc/local_map_t.hpp>
#include <bot_core/timestamp.h>

#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

using namespace std;

class State {
public:
  boost::shared_ptr<DeltaReceiver> mDeltaReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  bot_lcmgl_t* mLcmGl;

  State() {
    mDeltaReceiver.reset(new DeltaReceiver());
    mManager.reset(new MapManager());
    mLcm.reset(new lcm::LCM());
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "map-debug");
  }

  ~State() {
    bot_lcmgl_destroy(mLcmGl);
  }
};

class MapPublisher {
public:
  MapPublisher(State* iState) {
    mState = iState;
  }

  void operator()() {
    while (true) {
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      timer.expires_from_now(boost::posix_time::seconds(5));
      timer.wait();

      boost::shared_ptr<LocalMap> localMap = mState->mManager->getActiveMap();
      if (localMap == NULL) {
        continue;
      }
      std::vector<char> bytes;
      localMap->serialize(bytes);
      drc::local_map_t mapMessage;
      mapMessage.utime = bot_timestamp_now();
      mapMessage.id = localMap->getId();
      mapMessage.state_id = localMap->getStateId();
      mapMessage.size_bytes = bytes.size();
      mapMessage.data.insert(mapMessage.data.end(), bytes.begin(), bytes.end());
      mState->mLcm->publish("LOCAL_MAP", &mapMessage);
      cout << "Published map (" << bytes.size() << " bytes)" << endl;

      // publish as octomap (for debugging)
      octomap::raw_t raw = mState->mManager->getActiveMap()->getAsRaw();
      raw.utime = bot_timestamp_now();
      std::cout << "Publishing debug octomap (" << raw.length <<
        " bytes)..." << std::endl;
      mState->mLcm->publish("OCTOMAP", &raw);

      // write pgm height map (for debugging)
      LocalMap::HeightMap heightMap =
        mState->mManager->getActiveMap()->getAsHeightMap(3, 3.5);
      std::ofstream ofs("/home/antone/heightmap.txt");
      for (int i = 0; i < heightMap.mHeight; ++i) {
        for (int j = 0; j < heightMap.mWidth; ++j) {
          ofs << heightMap.mData[i*heightMap.mWidth+j] << " ";
        }
        ofs << std::endl;
      }
      std::cout << "Wrote debug height map" << std::endl;

      // publish height map via lcm
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

      if (false) {
        bot_lcmgl_t* lcmgl = mState->mLcmGl;
        bot_lcmgl_color3f(lcmgl, 1.0f, 0.5f, 0.0f);
        bot_lcmgl_point_size(lcmgl, 3.0f);
        bot_lcmgl_begin(lcmgl, GL_POINTS);
        LocalMap::PointCloud::Ptr cloud =
          mState->mManager->getActiveMap()->getAsPointCloud();
        cout << "lcmgl'ing " << cloud->size() << " points..." << endl;
        for (int k = 0; k < cloud->size(); ++k) {
          bot_lcmgl_vertex3f(lcmgl, cloud->points[k].x, cloud->points[k].y,
                             cloud->points[k].z);
        }
        bot_lcmgl_end(lcmgl);
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

  state.mManager->setMapResolution(0.01);
  state.mManager->setMapDimensions(Eigen::Vector3d(1e20, 1e20, 1e20));

  state.mDeltaReceiver->setLcm(state.mLcm);
  state.mDeltaReceiver->setManager(state.mManager);
  state.mDeltaReceiver->start();

  MapPublisher mapPublisher(&state);
  boost::thread thread(boost::ref(mapPublisher));

  while (0 == state.mLcm->handle());

  state.mDeltaReceiver->stop();

  return 0;
}
