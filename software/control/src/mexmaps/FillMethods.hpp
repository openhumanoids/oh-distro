#ifndef _mexmaps_FillMethods_hpp_
#define _mexmaps_FillMethods_hpp_

#include <memory>
#include <vector>
#include <set>
#include <Eigen/Geometry>

#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/drc/map_controller_command_t.hpp>

namespace maps {
  class BotWrapper;
  class DepthImageView;
}

namespace cv {
  class Mat;
}

namespace lcm {
  class ReceiveBuffer;
}

namespace mexmaps {

class FillMethods {
public:
  enum Method {
    MethodRobust,
    MethodRansac
  };

public:
  FillMethods(const std::shared_ptr<maps::BotWrapper>& iWrapper);

  float computeMedian(const Eigen::VectorXf& iData);

  Eigen::Vector3f fitHorizontalPlaneRobust
  (const std::vector<Eigen::Vector3f>& iPoints,
   const Eigen::Vector4f& iInitPlane=Eigen::Vector4f::Zero());

  Eigen::Vector3f fitHorizontalPlaneRansac
  (const std::vector<Eigen::Vector3f>& iPoints);

  Eigen::Vector3f fitHorizontalPlane
  (const std::vector<Eigen::Vector3f>& iPoints);

  int labelImage(cv::Mat& iMask, cv::Mat& oLabels);
  void labelRecurse(cv::Mat& iMask, const int iRow, const int iCol,
                    const int iLabel, cv::Mat& oLabels);
  void extractComponentsAndOutlines(const cv::Mat& iLabels,
                                    const int iNumLabels,
                                    std::vector<std::vector<int> >& oIndices,
                                    std::vector<std::set<int> >& oOutlines);

  void doFill(std::shared_ptr<maps::DepthImageView>& iView);
  void fillView(std::shared_ptr<maps::DepthImageView>& iView);
  void fillViewPlanar(std::shared_ptr<maps::DepthImageView>& iView);
  void fillUnderRobot(std::shared_ptr<maps::DepthImageView>& iView,
                      const Method iMethod);
  void fillContours(std::shared_ptr<maps::DepthImageView>& iView);
  void fillConnected(std::shared_ptr<maps::DepthImageView>& iView);
  void fillIterative(std::shared_ptr<maps::DepthImageView>& iView,
                     const int iMaxPasses=-1);
  void fillLevelPlaneFromFeet(std::shared_ptr<maps::DepthImageView>& iView);

protected:
  void onGround(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const bot_core::pose_t* iMessage);
  void onCommand(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_controller_command_t* iMessage);

protected:
  std::shared_ptr<maps::BotWrapper> mBotWrapper;
  Eigen::Vector4d mLatestGroundPlane;
  Eigen::Vector3d mLatestFeetPosition;
  int mMapMode;
  bool mDebug;
};

}

#endif
