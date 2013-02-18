#include <maps/ViewClient.hpp>

#include <lcmtypes/drc/map_image_t.hpp>

#include <drc_utils/Clock.hpp>

#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>

#include <ConciseArgs>

#include <iostream>

using namespace std;
using namespace maps;

class State : public ViewClient::Listener {
public:
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<ViewClient> mViewClient;
  std::string mHeightMapChannel;
  double mHeightMapResolution;
  float mMaxHeight;
  int64_t mViewId;

public:
  State() {
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    mViewClient.reset(new ViewClient());
    mViewClient->setLcm(mLcm);
    mViewClient->addListener(this);

    // defaults; these should be configured by command line args in main
    mHeightMapChannel = "HEIGHT_MAP";
    mHeightMapResolution = 0.1;
    mMaxHeight = 1e10;
    mViewId = 1;
  }

  ~State() {
  }

  void notifyCatalog(const bool iChanged) {}

  void notifyData(const int64_t iViewId) {
    //
    // compute heightmap
    //

    // get point cloud
    if (iViewId != mViewId) return;
    ViewClient::MapViewPtr mapView = mViewClient->getView(iViewId);
    if (mapView == NULL) return;
    MapView::HeightMap::Ptr heightMap =
      mapView->getAsHeightMap(mHeightMapResolution, mMaxHeight);

    // determine bounds
    

    // create array

    // accumulate data

    //
    // publish new heightmap via lcm
    //

    cout << "Publishing height map..." << endl;
    drc::map_image_t msg;

    // basic parameters
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.width = heightMap->mWidth;
    msg.height = heightMap->mHeight;
    msg.channels = 1;
    int sampleBytes = sizeof(float);
    msg.row_bytes = msg.width*sampleBytes;
    msg.total_bytes = msg.height*msg.row_bytes;
    msg.format = drc::map_image_t::FORMAT_GRAY_FLOAT32;
    msg.compression = drc::map_image_t::COMPRESSION_NONE;
    msg.data.resize(msg.total_bytes);
    memcpy(&msg.data[0], &heightMap->mData[0], msg.total_bytes);
    Eigen::Affine3f xform = heightMap->mTransform.inverse();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        msg.transform[i][j] = xform(i,j);
      }
    }

    // publish
    mLcm->publish(mHeightMapChannel, &msg);
  }
};

int main(const int iArgc, const char** iArgv) {
  // instantiate state
  State state;

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(state.mHeightMapChannel, "c", "heightmap_channel",
          "channel to publish height maps");
  opt.add(state.mHeightMapResolution, "r", "resolution",
          "approximate desired heightmap resolution");
  opt.add(state.mMaxHeight, "x", "max_height",
          "maximum height to consider when creating height map");
  opt.add(state.mViewId, "v", "view_id",
          "view id to use for creating heightmap data");
  opt.parse();

  // start running
  state.mViewClient->start();

  // handle lcm messages
  while (0 == state.mLcm->handle());

  return 0;
}
