#include <maps/MapManager.hpp>
#include <maps/SensorDataReceiver.hpp>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace std;

class State {
public:
  SensorDataReceiver mSensorDataReceiver;
  MapManager mManager;
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
      mState->mSensorDataReceiver.waitForData(data);
      mState->mManager.add(data.mTimestamp, data.mPointCloud, data.mPose);
      cout << "got data" << endl;

      /*
      // transform points to local frame
      SensorDataReceiver::PointCloud::Ptr points(new SensorDataReceiver::PointCloud);
      Eigen::Affine3f matx(data.mPose.cast<float>());
      matx = Eigen::Affine3f::Identity();
      pcl::transformPointCloud(*data.mPointCloud, *points, matx);

      // write points
      char fileName[256];
      sprintf(fileName, "/home/antone/map_pts_%.3d.pcd", mCounter);
      pcl::io::savePCDFileASCII(fileName, *points);
      ++mCounter;
      */
    }
  }

protected:
  State* mState;
  int mCounter;
};

int main(const int iArgc, const char** iArgv) {
  State state;
  cout << "STATE CONSTRUCTED" << endl;

  boost::shared_ptr<lcm::LCM> theLcm(new lcm::LCM());
  if (!theLcm->good()) {
    cerr << "Cannot create lcm instance." << endl;
    return -1;
  }

  BotParam* theParam = bot_param_new_from_file("/home/antone/drc/config/drc_robot.cfg");

  state.mSensorDataReceiver.setLcm(theLcm);
  state.mSensorDataReceiver.setBotParam(theParam);
  state.mSensorDataReceiver.setMaxBufferSize(100);
  state.mSensorDataReceiver.
    addChannel("WIDE_STEREO_POINTS",
               SensorDataReceiver::SensorTypePointCloud,
               "wide_stereo", "local");

  state.mManager.setMapResolution(0.1);
  state.mManager.setMapDimensions(Eigen::Vector3d(10,10,10));
  state.mManager.setDataBufferLength(1000);
  state.mManager.createMap(Eigen::Isometry3d::Identity());

  DataConsumer consumer(&state);
  boost::thread thread(consumer);
                                       
  while (0 == theLcm->handle());

  thread.join();

  return 0;
}
