#include <terrain-map/TerrainMap.hpp>

#include <chrono>
#include <thread>
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

using terrainmap::TerrainMap;

int main() {
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  std::shared_ptr<drc::LcmWrapper> lcmWrapper(new drc::LcmWrapper(lcm));
  std::shared_ptr<drc::BotWrapper> botWrapper(new drc::BotWrapper(lcm));
  TerrainMap terrainMap(botWrapper);

  terrainMap.setInfo(1000, "MAP_CONTROL_HEIGHT");
  terrainMap.sendSweepRequest(Eigen::Vector3d(-2,-5,-3),
                              Eigen::Vector3d(5,5,0.3), 0.03, 0.5);
  terrainMap.setFillPlane(Eigen::Vector4d(0,0,1,-123));
  terrainMap.overrideHeights(true);
  terrainMap.useFootPose(false);
  terrainMap.shouldFillMissing(true);

  lcmWrapper->startHandleThread();
  terrainMap.startListening();

  std::this_thread::sleep_for(std::chrono::seconds(3));

  Eigen::Vector3d normal;
  double height;
  terrainMap.getHeightAndNormal(1.0,2.0,height,normal);
  std::cout << "HEIGHT " << height << " NORMAL " << normal.transpose() << std::endl;

  auto data = terrainMap.getData();
  if (data == NULL) {
    std::cout << "IT IS NULL" << std::endl;
  }
  else {
    std::cout << data->mTransform.matrix() << std::endl;
  }

  terrainMap.stopListening();
  lcmWrapper->stopHandleThread();

  return 0;
}
