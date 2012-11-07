#include <iostream>
#include <maps/LocalMap.hpp>
#include <maps/MapWrapper.hpp>
#include <boost/thread.hpp>

using namespace std;

class SampleAction {
public:
  SampleAction(boost::shared_ptr<MapWrapper> iWrapper) {
    mWrapper = iWrapper;
  }

  void operator ()() {
    while (true) {
      LocalMap::PointCloud::Ptr cloud;
      mWrapper->lock();
      if (mWrapper->getMap() != NULL) {
        cloud = mWrapper->getMap()->getAsPointCloud();
      }
      mWrapper->unlock();

      cout << "Grabbed a point cloud of " << cloud->points.size() <<
        " points from octree" << endl;

      // ...do something interesting here...

      sleep(2);
    }
  }

protected:
  boost::shared_ptr<MapWrapper> mWrapper;
};

int main(const int iArgc, const char** iArgv) {
  boost::shared_ptr<lcm::LCM> theLcm(new lcm::LCM());
  if (!theLcm->good()) {
    cerr << "Cannot create lcm instance." << endl;
    return -1;
  }

  boost::shared_ptr<MapWrapper> wrapper(new MapWrapper());
  wrapper->setLcm(theLcm);
  wrapper->setMapChannel("LOCAL_MAP");
  wrapper->start();

  SampleAction action(wrapper);
  boost::thread thread(boost::ref(action));

  while (0 == theLcm->handle());

  wrapper->stop();

  return 0;
}
