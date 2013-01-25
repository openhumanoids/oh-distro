#include <unordered_map>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>

#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <maps/DataBlob.hpp>


#include <drc_utils/Clock.hpp>
#include <pcl/common/transforms.h>
#include <ConciseArgs>
#include <octomap/octomap.h>


using namespace std;
using namespace maps;

struct Worker {
  typedef boost::shared_ptr<Worker> Ptr;

  bool mActive;
  drc::map_request_t mRequest;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;

  void operator()() {
    mActive = true;
    while (true) {
      // wait for timer expiry
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      timer.expires_from_now(boost::posix_time::
                             milliseconds(1000/mRequest.frequency));
      timer.wait();
      std::cout << "Timer expired for view " << mRequest.view_id << std::endl;

      // get map
      LocalMap::Ptr localMap;
      if (mRequest.map_id == 0) {
        vector<int64_t> ids = mManager->getAllMapIds(true);
        if (ids.size() > 0) {
          localMap = mManager->getMap(ids.back());
        }
      }
      else {
        localMap = mManager->getMap(mRequest.map_id);
      }
      if (localMap == NULL) {
        continue;
      }

      // get bounds
      LocalMap::SpaceTimeBounds bounds;
      bounds.mMinTime = mRequest.time_min;
      bounds.mMaxTime = mRequest.time_max;
      bounds.mPlanes.resize(mRequest.num_clip_planes);
      for (int i = 0; i < bounds.mPlanes.size(); ++i) {
        bounds.mPlanes[i] =
          Eigen::Vector4f(mRequest.clip_planes[i][0],
                          mRequest.clip_planes[i][1],
                          mRequest.clip_planes[i][2],
                          mRequest.clip_planes[i][3]);
      }

      // get and publish octree
      if (mRequest.type == drc::map_request_t::OCTREE) {
        LocalMap::Octree octree =
          localMap->getAsOctree(mRequest.resolution, false,
                                Eigen::Vector3f(0,0,0), bounds);
        std::cout << "Publishing octomap..." << std::endl;
        std::ostringstream oss;
        octree.mTree->writeBinaryConst(oss);
        std::string str = oss.str();
        Eigen::Isometry3f matx = octree.mTransform.inverse();
        drc::map_octree_t octMsg;
        octMsg.utime = drc::Clock::instance()->getCurrentTime();
        octMsg.map_id = localMap->getId();
        octMsg.view_id = mRequest.view_id;
        octMsg.num_bytes = str.size();
        octMsg.data.resize(octMsg.num_bytes);
        std::copy(str.begin(), str.end(), octMsg.data.begin());
        Eigen::Quaternionf q(matx.linear());
        Eigen::Vector3f t(matx.translation());
        octMsg.transform.translation.x = t[0];
        octMsg.transform.translation.y = t[1];
        octMsg.transform.translation.z = t[2];
        octMsg.transform.rotation.w = q.w();
        octMsg.transform.rotation.x = q.x();
        octMsg.transform.rotation.y = q.y();
        octMsg.transform.rotation.z = q.z();
        mLcm->publish("MAP_OCTREE", &octMsg);
        std::cout << "Sent octree at " << octMsg.num_bytes << " bytes" << std::endl;
      }

      else if (mRequest.type == drc::map_request_t::CLOUD) {

        // get point cloud
        maps::PointCloud::Ptr cloud =
          localMap->getAsPointCloud(mRequest.resolution, bounds);

        // find extrema of cloud and transform points
        maps::PointCloud::PointType maxPoint, minPoint;
        pcl::getMinMax3D(*cloud, minPoint, maxPoint);
        Eigen::Vector3f offset(minPoint.x, minPoint.y, minPoint.z);
        Eigen::Vector3f scale(maxPoint.x-minPoint.x, maxPoint.y-minPoint.y,
                              maxPoint.z-minPoint.z);
        scale /= 255;
        Eigen::Affine3f xform = Eigen::Affine3f::Identity();
        for (int i = 0; i < 3; ++i) {
          xform(i,i) = scale[i];
          xform(i,3) = offset[i];
        }
        xform = xform.inverse();
        pcl::transformPointCloud(*cloud, *cloud, xform);

        // store to blob
        int totalSize = cloud->points.size()*3;
        std::vector<uint8_t> data(totalSize*sizeof(float));
        float* ptr = (float*)(&data[0]);
        for (int i = 0; i < cloud->size(); ++i) {
          maps::PointCloud::PointType pt = cloud->points[i];
          ptr[i*3+0] = pt.x;
          ptr[i*3+1] = pt.y;
          ptr[i*3+2] = pt.z;
        }
        DataBlob::Spec spec;
        spec.mDimensions.push_back(3);
        spec.mDimensions.push_back(cloud->size());
        spec.mStrideBytes.push_back(sizeof(float));
        spec.mStrideBytes.push_back(3*sizeof(float));
        spec.mCompressionType = DataBlob::CompressionTypeNone;
        spec.mDataType = DataBlob::DataTypeFloat32;
        DataBlob blob;
        blob.setData(data, spec);

        // compress and convert
        blob.convertTo(DataBlob::CompressionTypeZlib,
                       DataBlob::DataTypeUint8);

        // transmit blob
        drc::map_cloud_t msgCloud;
        msgCloud.utime = drc::Clock::instance()->getCurrentTime();
        msgCloud.map_id = localMap->getId();
        msgCloud.view_id = mRequest.view_id;
        msgCloud.blob.utime = msgCloud.utime;
        msgCloud.blob.num_dims = 2;
        msgCloud.blob.dimensions.resize(2);
        msgCloud.blob.stride_bytes.resize(2);
        msgCloud.blob.dimensions[0] = spec.mDimensions[0];
        msgCloud.blob.dimensions[1] = spec.mDimensions[1];
        msgCloud.blob.stride_bytes[0] = spec.mStrideBytes[0];
        msgCloud.blob.stride_bytes[1] = spec.mStrideBytes[1];
        msgCloud.blob.compression = drc::map_blob_t::ZLIB;
        msgCloud.blob.data_type = drc::map_blob_t::UINT8;
        msgCloud.blob.num_bytes = data.size();
        msgCloud.blob.data = data;
        Eigen::Affine3f xformInv = xform.inverse();
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            msgCloud.transform[i][j] = xformInv(i,j);
          }
        }
        mLcm->publish("MAP_CLOUD", &msgCloud);
      }

      // one-shot request has 0 frequency
      if (fabs(mRequest.frequency) < 1e-6) {
        break;
      }
    }
    mActive = false;
  }
};

typedef std::unordered_map<int64_t,Worker::Ptr> WorkerMap;

class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  WorkerMap mWorkers;

  lcm::Subscription* mRequestSubscription;

  State() {
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mManager.reset(new MapManager());
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    mSensorDataReceiver->setLcm(mLcm);
    mRequestSubscription = NULL;
  }

  ~State() {
    mLcm->unsubscribe(mRequestSubscription);
  }

  void onRequest(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_request_t* iMessage) {
    addWorker(*iMessage);
  }

  void addWorker(const drc::map_request_t& iRequest) {
    WorkerMap::const_iterator item = mWorkers.find(iRequest.view_id);
    if (item != mWorkers.end()) {
      if (!item->second->mActive) {
        boost::thread thread(*item->second);
      }
    }
    else {
      Worker::Ptr worker(new Worker());
      worker->mActive = false;
      worker->mLcm = mLcm;
      worker->mManager = mManager;
      worker->mRequest = iRequest;
      mWorkers[iRequest.view_id] = worker;
      boost::thread thread(*worker);
    }
  }
};

class DataConsumer {
public:
  DataConsumer(State* iState) {
    mState = iState;
  }

  void operator()() {
    while(true) {
      maps::PointSet data;
      if (mState->mSensorDataReceiver->waitForData(data)) {
        mState->mManager->addData(data);
      }
    }
  }

protected:
  State* mState;
};

int main(const int iArgc, const char** iArgv) {

  // instantiate state object
  State state;

  // parse arguments
  string laserChannel = "ROTATING_SCAN";
  float publishPeriod = 3;
  float defaultResolution = 0.1;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(laserChannel, "l", "laser_channel",
          "laser channel to use in map creation");
  opt.add(publishPeriod, "p", "publish_period",
          "interval between map publications, in s");
  opt.add(defaultResolution, "r", "resolution",
          "resolution of default contextual map, in m");
  opt.parse();
  state.mSensorDataReceiver->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");

  // set up remaining parameters
  state.mSensorDataReceiver->setMaxBufferSize(100);
  LocalMap::Spec spec;
  spec.mPointBufferSize = 5000;
  spec.mActive = true;
  spec.mBoundMin = Eigen::Vector3f(-5,-5,-5);
  spec.mBoundMax = Eigen::Vector3f(5,5,5);
  int id = state.mManager->createMap(spec);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_REQUEST", &State::onRequest, &state);

  // start running data receiver
  BotParam* theParam =
    bot_param_new_from_server(state.mLcm->getUnderlyingLCM(), 0);
  state.mSensorDataReceiver->setBotParam(theParam);
  state.mSensorDataReceiver->start();

  // start consuming data
  DataConsumer consumer(&state);
  boost::thread consumerThread(consumer);

  // start publishing data
  drc::map_request_t request;
  request.map_id = 1;
  request.view_id = 1;
  request.type = drc::map_request_t::OCTREE;
  request.resolution = defaultResolution;
  request.frequency = 1.0/publishPeriod;
  request.time_min = -1;
  request.time_max = -1;
  request.num_clip_planes = 0;
  state.addWorker(request);

  // main lcm loop
  while (0 == state.mLcm->handle());

  consumerThread.join();

  return 0;
}
