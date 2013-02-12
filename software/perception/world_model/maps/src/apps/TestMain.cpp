#include <maps/ViewClient.hpp>
#include <maps/MapView.hpp>
#include <lcm/lcm-cpp.hpp>

#include <maps/VoxelGrid.hpp>
#include <chrono>
#include <maps/SensorDataReceiver.hpp>
#include <boost/thread.hpp>
#include <fstream>

using namespace maps;
using namespace std;

struct BoxOperator {
  int64_t mCounter;
  int64_t mCounter2;

  BoxOperator() {
    mCounter = 0;
    mCounter2 = 0;
  }

  bool operator()(const Eigen::Vector3i& iP1, const Eigen::Vector3i& iP2,
                  VoxelGrid<int8_t>& iGrid) {
    /*
    std::cout << "(" << iP1.transpose() << ")    (" << iP2.transpose() << ")";
    if (iP1==iP2) {
      std::cout << " SAME";
    }
    std::cout << std::endl;
    */

    ++mCounter;
    return true;
  }
};

struct RayOperator {
  std::vector<Eigen::Vector3i> mPoints;

  void operator()(const Eigen::Vector3i& iPoint,
                  VoxelGrid<int8_t>& iGrid) {
    mPoints.push_back(iPoint);
    // TODO std::cout << iPoint.transpose() << std::endl;
  }
};



class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<lcm::LCM> mLcm;

  State() {
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mLcm.reset(new lcm::LCM());
    mSensorDataReceiver->setLcm(mLcm);
  }

  ~State() {
  }
};



class DataConsumer {
public:
  DataConsumer(State* iState) {
    mState = iState;
    mStream.open("/home/antone/lidar_log.txt");
  }

  void operator()() {
    while(true) {
      maps::PointSet data;
      if (mState->mSensorDataReceiver->waitForData(data)) {
        for (int i = 0; i < 3; ++i) {
          mStream << data.mCloud->sensor_origin_[i] << " ";
        }
        mStream << data.mCloud->sensor_orientation_.w() << " " <<
          data.mCloud->sensor_orientation_.x() << " " <<
          data.mCloud->sensor_orientation_.y() << " " <<
          data.mCloud->sensor_orientation_.z() << " ";
        for (int i = 0; i < data.mCloud->size(); ++i) {
          mStream << data.mCloud->points[i].x << " " <<
            data.mCloud->points[i].y << " " <<
            data.mCloud->points[i].z << " ";
        }
        mStream << std::endl;
      }
    }
  }

protected:
  State* mState;
  std::ofstream mStream;
};



int main() {

  // this runs a sensor capture loop
  State state;
  std::string laserChannel("ROTATING_SCAN");
  state.mSensorDataReceiver->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");
  BotParam* theParam =
    bot_param_new_from_server(state.mLcm->getUnderlyingLCM(), 0);
  state.mSensorDataReceiver->setBotParam(theParam);
  state.mSensorDataReceiver->start();
  DataConsumer consumer(&state);
  boost::thread consumerThread(boost::ref(consumer));
  while (0 == state.mLcm->handle());
  consumerThread.join();
  return 0;




  VoxelGrid<int8_t> grid;
  grid.setDimensions(1024,1024,1024);

  auto t1 = chrono::high_resolution_clock::now();  

  const int whichOne = 2;

  if (whichOne==0) {
    BoxOperator op;
    grid.operateRecursive(op);
    std::cout << "COUNTER " << op.mCounter << std::endl;
    std::cout << "COUNTER2 " << op.mCounter2 << std::endl;
  }

  else if (whichOne==1) {
    auto dur = std::chrono::high_resolution_clock::now().time_since_epoch();
    std::default_random_engine generator(dur.count());
    std::uniform_int_distribution<int64_t> distribution;
    int64_t plane[4];
    for (int i = 0; i < 4; ++i) {
      plane[i] = distribution(generator)/1000000000;
    }
    const Eigen::Vector3i dims = grid.getDimensions();
    /*
    const int64_t nx = dims[0];
    const int64_t ny = dims[1];
    const int64_t nz = dims[2];
    */
    const int64_t nx = 1024;
    const int64_t ny = 1024;
    const int64_t nz = 1024;
    int64_t total = 0;
    for (int64_t i = 0; i < nx; ++i) {
      for (int64_t j = 0; j < ny; ++j) {
        for (int64_t k = 0; k < nz; ++k) {
          int64_t dist = i*plane[0] + j*plane[1] + k*plane[2] + plane[3];
          total += dist;
        }
      }
    }
    std::cout << "TOTAL= " << total << std::endl;
  }

  else if (whichOne==2) {
    RayOperator op;
    const int kNumScans = 40;
    const int kNumPointsPerScan = 1024;
    const int kNumRaysPerPoint = 1;
    for (int i = 0; i < kNumScans; ++i) {
      op.mPoints.reserve(kNumPointsPerScan*kNumRaysPerPoint);
      for (int j = 0; j < kNumPointsPerScan; ++j) {
        for (int k = 0; k < kNumRaysPerPoint; ++k) {
          grid.traceRay(Eigen::Vector3f(500,-3,500),
                        Eigen::Vector3f(600,1200,300), op);
        }
      }
      //std::cout << "NUM POINTS: " << op.mPoints.size() << std::endl;
      op.mPoints.clear();
    }
  }

  else if (whichOne==3) {
    typedef int64_t Type;

    auto dur = std::chrono::high_resolution_clock::now().time_since_epoch();
    std::default_random_engine generator(dur.count());
    std::uniform_int_distribution<Type> distribution;
    Eigen::Transform<Type,3,Eigen::Affine> m;
    m.setIdentity();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        m(i,j) = distribution(generator)/1000000000;
      }
    }
    std::cout << m.matrix() << std::endl;

    int nscans = 40;
    Type nx = 1024;
    Type ny = 1024;
    Type nz = 1024;
    Type counter = 0;
    Type sx(0), sy(0), sz(0);
    for (int s = 0; s < nscans; ++s) {
      for (Type i = 0; i < nx; ++i) {
        for (Type j = 0; j < ny; ++j) {
          for (Type k = 0; k < nz; ++k) {
            Type x = i*m(0,0) + j*m(0,1) + k*m(0,2) + m(0,3);
            Type y = i*m(1,0) + j*m(1,1) + k*m(1,2) + m(1,3);
            Type z = i*m(2,0) + j*m(2,1) + k*m(2,2) + m(2,3);
            //counter += i*i + j*j + k*k;
            sx += x;
            sy += y;
            sz += z;
          }
        }
      }
    }
    cout << "SUMS " << sx << " " << sy << " " << sz << endl;
    cout << "COUNTER " << counter << std::endl;
  }
  auto t2 = chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1);
  cout << "Elapsed time: " << duration.count()/1e3 << " seconds" << std::endl;

  return 0;

  boost::shared_ptr<ViewClient> client(new ViewClient());
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  client->setLcm(lcm);
  client->start();

  for (int i = 0; i < 5; ++i) {
    sleep(1);
    std::vector<MapView::Ptr> views = client->getAllViews();
    maps::PointCloud::Ptr cloud(new maps::PointCloud());
    for (int k = 0; k < views.size(); ++k) {
      maps::PointCloud::Ptr cloudCur = views[k]->getAsPointCloud();
      *cloud += *cloudCur;
    }
    cout << "DONE " << i << endl;
  }
}
