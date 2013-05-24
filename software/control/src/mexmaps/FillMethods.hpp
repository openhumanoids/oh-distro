#ifndef _mexmaps_FillMethods_hpp_
#define _mexmaps_FillMethods_hpp_

#include <memory>
#include <vector>
#include <Eigen/Geometry>

namespace maps {
  class BotWrapper;
  class DepthImageView;
}

namespace cv {
  class Mat;
}

namespace mexmaps {

class FillMethods {
public:
  FillMethods(const std::shared_ptr<maps::BotWrapper>& iWrapper);

  float computeMedian(const Eigen::VectorXf& iData);

  Eigen::Vector3f fitHorizontalPlaneRobust
  (const std::vector<Eigen::Vector3f>& iPoints,
   const Eigen::Vector4f& iInitPlane=Eigen::Vector4f::Zero());

  void labelImage(cv::Mat& iMask, cv::Mat& oLabels);
  void labelRecurse(cv::Mat& iMask, const int iRow, const int iCol,
                    const int iLabel, cv::Mat& oLabels);

  void fillView(std::shared_ptr<maps::DepthImageView>& iView);
  void fillViewPlanar(std::shared_ptr<maps::DepthImageView>& iView);
  void fillUnderRobot(std::shared_ptr<maps::DepthImageView>& iView);
  void fillContours(std::shared_ptr<maps::DepthImageView>& iView);
  void fillIterative(std::shared_ptr<maps::DepthImageView>& iView,
                     const int iMaxPasses=-1);

protected:
  std::shared_ptr<maps::BotWrapper> mBotWrapper;
};

}

#endif
