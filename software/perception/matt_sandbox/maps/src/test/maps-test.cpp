#include <maps/MapManager.hpp>

#include <pcl/io/io.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/pointcloud2_t.hpp>

using namespace std;

class State {
public:
  void onCollection(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const drc::pointcloud2_t* iMessage) {
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pointCloud.width = iMessage->width;
    pointCloud.height = iMessage->height;
    pointCloud.points.resize(iMessage->width * iMessage->height);
    pointCloud.is_dense = false;
    uint8_t* cloudData = reinterpret_cast<uint8_t*>(&pointCloud.points[0]);
    const uint8_t* messageData =
      reinterpret_cast<const uint8_t*>(&iMessage->data[0]);
    memcpy(cloudData, messageData, iMessage->data_nbytes);
    pcl::PointCloud<pcl::PointXYZ> newCloud;
    pcl::copyPointCloud(pointCloud, newCloud);
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    mManager.add(iMessage->utime, newCloud, transform);
    cout << "GOT COLLECTION" << endl;
  }

public:
  MapManager mManager;
};

int main(const int iArgc, const char** iArgv) {
  State state;
  state.mManager.setMapResolution(0.1);

  // set up lcm instance
  // TODO: should spawn thread
  lcm::LCM theLcm;
  if (!theLcm.good()) {
    cerr << "Cannot create lcm instance." << endl;
    return -1;
  }
  theLcm.subscribe("WIDE_STEREO_POINTS", &State::onCollection, &state);
  while (true) {
    theLcm.handle();
  }

  return 0;
}
