#ifndef _mexmaps_ViewClientWrapper_hpp_
#define _mexmaps_ViewClientWrapper_hpp_

#include <memory>
#include <string>
#include <Eigen/Geometry>

namespace lcm {
  class LCM;
}

namespace maps {
  class BotWrapper;
  class ViewClient;
  class ViewBase;
  class DepthImageView;
}

namespace mexmaps {

class MapHandle;

struct ViewClientWrapper {
  struct Listener;

  int mId;
  std::shared_ptr<maps::ViewClient> mViewClient;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<maps::BotWrapper> mBotWrapper;
  std::shared_ptr<Listener> mListener;
  int mHeightMapViewId;
  std::shared_ptr<MapHandle> mHandle;

  int mNormalRadius;
  bool mShouldFill;

  ViewClientWrapper(const int iId, const std::shared_ptr<lcm::LCM>& iLcm);
  ~ViewClientWrapper();

  void setHeightMapChannel(const std::string& iChannel, const int iViewId);

  bool start();
  bool stop();

  std::shared_ptr<maps::ViewBase> getView();

protected:
  void requestHeightMap();
  Eigen::Vector3f fitHorizontalPlaneRobust
    (const std::vector<Eigen::Vector3f>& iPoints,
     const Eigen::Vector4f& iInitPlane=Eigen::Vector4f::Zero());
  void fillView(std::shared_ptr<maps::DepthImageView>& iView);
  void fillViewPlanar(std::shared_ptr<maps::DepthImageView>& iView);
  void fillUnderRobot(std::shared_ptr<maps::DepthImageView>& iView);
};

}

#endif
