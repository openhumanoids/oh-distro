#ifndef _maps_MeshRenderer_h_
#define _maps_MeshRenderer_h_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace lcm {
  class LCM;
}

typedef struct _BotParam BotParam;
typedef struct _BotFrames BotFrames;

namespace maps {

class MeshRenderer {
public:
  enum ColorMode {
    ColorModeFlat,     // single solid color
    ColorModeHeight,   // pseudocolor by z
    ColorModeRange,    // pseudocolor by range
    ColorModeNormal,   // pseudocolor by normal vector
    ColorModeTexture,  // general texture image
    ColorModeCamera,   // bot_image_t from camera channel
    ColorModeMap       // pseudocolor via colormap
  };

  enum MeshMode {
    MeshModePoints,      // individual points
    MeshModeWireframe,   // unfilled wireframe mesh
    MeshModeFilled,      // filled mesh, solid color
    MeshModeShaded       // shaded solid mesh
  };

public:
  MeshRenderer();
  ~MeshRenderer();

  void setBotObjects(const std::shared_ptr<lcm::LCM> iLcm,
                     const BotParam* iParam, const BotFrames* iFrames);
  void addCameraChannel(const std::string& iChannel,
                        const bool iMultipleImages=false);
  void setActiveCameraChannel(const std::string& iChannel);
  void setColorMode(const ColorMode& iMode);
  void setMeshMode(const MeshMode& iMode);

  void setColor(const float iR, const float iG, const float iB);
  void setScaleRange(const float iMinZ, const float iMaxZ);
  void setColorAlpha(const float iAlpha);
  void setPointSize(const float iSize);
  void setLineWidth(const float iWidth);
  void setRangeOrigin(const Eigen::Vector3f& iOrigin);
  void setNormalZero(const Eigen::Vector3f& iZero);

  void setTexture(const int iWidth, const int iHeight, const int iStrideBytes,
                  const int iFormat, const uint8_t* iData,
                  const Eigen::Projective3f& iTransform=
                  Eigen::Projective3f::Identity());

  void setData(const std::vector<Eigen::Vector3f>& iVertices,
               const std::vector<Eigen::Vector3f>& iNormals,
               const std::vector<Eigen::Vector3i>& iFaces,
               const Eigen::Projective3f& iTransform=
               Eigen::Affine3f::Identity());

  void setData(const std::vector<Eigen::Vector3f>& iVertices,
               const std::vector<Eigen::Vector3i>& iFaces,
               const Eigen::Projective3f& iTransform=
               Eigen::Projective3f::Identity());

  void draw();

protected:
  struct InternalState;
  std::shared_ptr<InternalState> mState;
};

}

#endif
