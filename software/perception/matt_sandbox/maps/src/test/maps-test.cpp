#include <maps/SpatialQueryWrapper.hpp>
#include <maps/SpatialQuery.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/progress.hpp>
#include <lcm/lcm-cpp.hpp>

using namespace std;

struct Worker {
  Worker(SpatialQueryWrapper* iWrapper) {
    mWrapper = iWrapper;
  }

  void operator()() {
    while (true) {
      sleep(1);
      {
        boost::progress_timer timer;
        mWrapper->lock();
        int total = 0;
        for (int i = 0; i < 100000; ++i) {
          Eigen::Vector3d pt(1.74,-1.17,1.53);
          Eigen::Vector3d outPoint, outNormal;
          if (mWrapper->query()->getClosest(pt, outPoint, outNormal)) {
            ++total;
          }
        }
        mWrapper->unlock();
        cout << "succeeded for " << total << " queries" << endl;
      }
    }
  }

  SpatialQueryWrapper* mWrapper;
};

int main(const int iArgc, const char** iArgv) {
  boost::shared_ptr<lcm::LCM> theLcm(new lcm::LCM());
  if (!theLcm->good()) {
    cerr << "Cannot create lcm object" << endl;
    return -1;
  }

  SpatialQueryWrapper wrapper;
  wrapper.setLcm(theLcm);
  wrapper.setMapChannel("LOCAL_MAP");
  wrapper.start();

  Worker worker(&wrapper);
  boost::thread thread(boost::ref(worker));

  while (0 == theLcm->handle());

  wrapper.stop();

  return 0;
}
