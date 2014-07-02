#ifndef _maps_LidarAccumulator_hpp_
#define _maps_LidarAccumulator_hpp_

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Geometry>

namespace drc {
  class BotWrapper;
}

namespace maps {
class LidarAccumulator {
public:
  LidarAccumulator();
  ~LidarAccumulator();

  bool setCameraChannel(const std::string& iChannel);
  bool setLidarChannel(const std::string& iChannel);
  bool setBotWrapper(const std::shared_ptr<drc::BotWrapper>& iBotWrapper);
  bool setRangeLimits(const float iRangeMin, const float iRangeMax);

  bool start();
  bool stop();
  bool isRunning() const;

  bool getPointCloud(const double iRevolutions, const double iStartAngle,
                     std::vector<Eigen::Vector3f>& oCloud);

protected:
  class Helper;
  std::shared_ptr<Helper> mHelper;
};
}

#endif
