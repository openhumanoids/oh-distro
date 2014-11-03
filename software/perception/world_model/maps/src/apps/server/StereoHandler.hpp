#ifndef _maps_StereoHandler_hpp_
#define _maps_StereoHandler_hpp_

#include <string>
#include <memory>
#include <vector>
#include <Eigen/Geometry>

namespace drc {
  class map_request_t;
}

namespace maps {

class BotWrapper;
class DepthImageView;

class StereoHandler {


public:
  StereoHandler(const std::shared_ptr<BotWrapper>& iBotWrapper,
                const std::string& iCameraBaseName);
  ~StereoHandler();

  bool isGood() const;

  std::shared_ptr<DepthImageView>
  getDepthImageView(const std::vector<Eigen::Vector4f>& iBoundPlanes);

  std::shared_ptr<DepthImageView>
  getDepthImageView(const drc::map_request_t& iRequest);

private:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

}

#endif
