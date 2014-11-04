#ifndef _maps_FusedDepthHandler_hpp_
#define _maps_FusedDepthHandler_hpp_

#include <string>
#include <memory>


namespace lcm {
  class Subscription;
}

namespace drc {
  class map_request_t;
}

namespace maps {

class BotWrapper;
class DepthImageView;

class FusedDepthHandler {

public:
  FusedDepthHandler(const std::shared_ptr<BotWrapper>& iBotWrapper);
  ~FusedDepthHandler();

  void setDepthChannel(const std::string& iChannel);
  void setCameraChannel(const std::string& iChannel);
  void setMedianFilter(const int iKernelRadius);

  void start();
  void stop();

  std::shared_ptr<DepthImageView> getLatest() const;

  std::shared_ptr<DepthImageView>
  getLatest(const drc::map_request_t& iRequest) const;

protected:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

}

#endif
