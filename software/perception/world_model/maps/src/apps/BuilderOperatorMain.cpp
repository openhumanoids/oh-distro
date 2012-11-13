#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/DeltaReceiver.hpp>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/local_map_t.hpp>
#include <bot_core/timestamp.h>

#include <ConciseArgs>

using namespace std;

class State {
public:
  boost::shared_ptr<DeltaReceiver> mDeltaReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;

  State() {
    mDeltaReceiver.reset(new DeltaReceiver());
    mManager.reset(new MapManager());
    mLcm.reset(new lcm::LCM());
    mDeltaReceiver->setManager(mManager);
    mDeltaReceiver->setLcm(mLcm);
  }

  ~State() {
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

      // publish map
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
      octomap::raw_t raw = localMap->getAsRaw();
      raw.utime = bot_timestamp_now();
      std::cout << "Publishing debug octomap (" << raw.length <<
        " bytes)..." << std::endl;
      mState->mLcm->publish("OCTOMAP", &raw);
    }
  }

protected:
  State* mState;
};

int main(const int iArgc, const char** iArgv) {
  // create state instance
  State state;

  // parse command line options
  string paramsChannel = "MAP_PARAMS";
  string updateChannel = "MAP_UPDATE";
  string ackChannel = "MAP_ACK";
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(paramsChannel, "c", "create_channel",
          "channel for receiving map create messages");
  opt.add(paramsChannel, "u", "update_channel",
          "channel for receiving map update messages");
  opt.add(paramsChannel, "a", "ack_channel",
          "channel for sending ack messages");
  state.mDeltaReceiver->setParamsChannel(paramsChannel);
  state.mDeltaReceiver->setUpdateChannel(updateChannel);
  state.mDeltaReceiver->setAckChannel(ackChannel);

  // start up receiver
  state.mDeltaReceiver->start();

  // start up publisher thread
  MapPublisher mapPublisher(&state);
  boost::thread thread(boost::ref(mapPublisher));

  // main lcm loop
  while (0 == state.mLcm->handle());

  state.mDeltaReceiver->stop();

  return 0;
}
