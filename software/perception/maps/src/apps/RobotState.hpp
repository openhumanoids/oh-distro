#ifndef _drc_RobotStateWrapper_hpp_
#define _drc_RobotStateWrapper_hpp_

#include <memory>
#include <string>
#include <Eigen/Geometry>

namespace lcm {
  class LCM;
}

namespace maps {

class RobotState {
private:
  struct Helper;

public:
  RobotState(const std::shared_ptr<lcm::LCM>& iLcm);
  ~RobotState();

  bool getPose(const std::string& iLink, Eigen::Quaternionf& oOrientation,
               Eigen::Vector3f& oPosition) const;

  std::shared_ptr<Helper> mHelper;
};

}

#endif
