#ifndef _maps_MeshRenderer_h_
#define _maps_MeshRenderer_h_

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace lcm {
  class LCM;
}

namespace maps {

class MeshRenderer {
public:
  enum ColorMode {
    ColorModeFlat,
    ColorModeHeight,
    ColorModeTexture,
    ColorModeCamera
  };

  enum MeshMode {
    MeshModePoints,
    MeshModeWireframe,
    MeshModeFilled,
    MeshModeShaded
  };

public:
  MeshRenderer();
  ~MeshRenderer();

  void setLcm(const boost::shared_ptr<lcm::LCM> iLcm);
  void setCameraChannel(const std::string& iChannel);
  void setColorMode(const ColorMode& iMode);
  void setMeshMode(const MeshMode& iMode);

  void setColor(const float iR, const float iG, const float iB);
  void setHeightScale(const float iMinZ, const float iMaxZ);
  void setColorAlpha(const float iAlpha);
  void setPointSize(const float iSize);
  // TODO void setTexture();
  void setMesh(const std::vector<Eigen::Vector3f>& iVertices,
               const std::vector<Eigen::Vector3i>& iFaces,
               const Eigen::Affine3f& iTransform=Eigen::Affine3f::Identity());

  void draw();

protected:
  struct InternalState;
  boost::shared_ptr<InternalState> mState;
};

}

#endif
