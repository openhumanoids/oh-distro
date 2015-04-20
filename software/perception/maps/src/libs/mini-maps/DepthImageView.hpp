#ifndef _maps_DepthImageView_hpp_
#define _maps_DepthImageView_hpp_

#include "ViewBase.hpp"

namespace maps {

class DepthImage;

class DepthImageView : public ViewBase {
public:
  typedef std::shared_ptr<DepthImageView> Ptr;

  enum NormalMethod {
    NormalMethodTriangle,
    NormalMethodLeastSquares,
    NormalMethodRobustKernel,
    NormalMethodSampleConsensus,
    NormalMethodZ
  };

public:
  DepthImageView();
  ~DepthImageView();

  void setSize(const int iWidth, const int iHeight);
  void set(const DepthImage& iImage);
  std::shared_ptr<DepthImage> getDepthImage() const;

  void setNormalRadius(const int iRadiusPixels);
  void setNormalMethod(const NormalMethod iMethod);

  const Type getType() const;
  ViewBase::Ptr clone() const;
  bool getClosest(const Eigen::Vector3f& iPoint,
                  Eigen::Vector3f& oPoint, Eigen::Vector3f& oNormal) const;

protected:
  const std::vector<float>& getInnerData(const int iType) const;
  bool interpolate(const float iX, const float iY, float& oDepth) const;
  bool unproject(const Eigen::Vector3f& iPoint, Eigen::Vector3f& oPoint,
                 Eigen::Vector3f& oNormal) const;

  static bool fitPlaneSac(const Eigen::MatrixX3f& iPoints,
                          Eigen::Vector4f& oPlane);
  static bool fitPlaneRobust(const Eigen::MatrixX3f& iPoints,
                             Eigen::Vector4f& oPlane);
  static bool fitPlaneRobust(const Eigen::MatrixX3f& iPoints,
                             const Eigen::Vector3f& iPointOnPlane,
                             Eigen::Vector4f& oPlane);
  static bool fitPlane(const Eigen::MatrixX3f& iPoints,
                       const Eigen::VectorXf& iWeights,
                       Eigen::Vector4f& oPlane);
  static bool fitPlane(const Eigen::MatrixX3f& iPoints,
                       const Eigen::Vector3f& iPointOnPlane,
                       const Eigen::VectorXf& iWeights,
                       Eigen::Vector4f& oPlane);

protected:
  std::shared_ptr<DepthImage> mImage;
  int mNormalRadius;
  NormalMethod mNormalMethod;
};

}

#endif
