#include <maps/ViewClient.hpp>
#include <maps/MapView.hpp>
#include <lcm/lcm-cpp.hpp>

using namespace maps;
using namespace std;

int main() {
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
