#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/DeltaReceiver.hpp>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/raw_t.hpp>
#include <lcmtypes/drc/heightmap_t.hpp>
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

      if (mState->mManager->getActiveMap() == NULL) {
        continue;
      }
      std::vector<char> bytes;
      mState->mManager->getActiveMap()->serialize(bytes);
      bot_core::raw_t mapBytes;
      mapBytes.utime = bot_timestamp_now();
      mapBytes.length = bytes.size();
      mapBytes.data.insert(mapBytes.data.end(), bytes.begin(), bytes.end());
      mState->mLcm->publish("LOCAL_MAP", &mapBytes);
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
      heightMapMsg.heights = heightMap.mData;
      Eigen::Affine3d::TranslationPart trans =
        heightMap.mTransformToLocal.translation();
      heightMapMsg.transform_to_local.translation.x = trans[0];
      heightMapMsg.transform_to_local.translation.y = trans[1];
      heightMapMsg.transform_to_local.translation.z = trans[2];
      Eigen::Quaterniond quat(heightMap.mTransformToLocal.rotation());
      heightMapMsg.transform_to_local.rotation.x = quat.x();
      heightMapMsg.transform_to_local.rotation.y = quat.y();
      heightMapMsg.transform_to_local.rotation.z = quat.z();
      heightMapMsg.transform_to_local.rotation.w = quat.w();
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
