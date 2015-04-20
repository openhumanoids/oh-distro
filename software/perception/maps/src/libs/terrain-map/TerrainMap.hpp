#ifndef _terrain_map_TerrainMap_hpp_
#define _terrain_map_TerrainMap_hpp_

#include <memory>
#include <string>
#include <Eigen/Geometry>

namespace drc {
  class BotWrapper;
}

namespace terrainmap {

class TerrainMap {
public:
  enum NormalMethod {
    NormalMethodOverride,         // use fill plane for normals
    NormalMethodTriangle,         // use triangular mesh facet normal
    NormalMethodLeastSquares,     // least squares plane estimate
    NormalMethodRobustKernel,     // iterative plane fit with robust kernel
    NormalMethodSampleConsensus,  // ransac plane estimate
  };

  struct TerrainData {
    std::vector<float> mHeights;
    int mWidth;
    int mHeight;
    Eigen::Projective3d mTransform;
  };

public:
  TerrainMap(const std::shared_ptr<drc::BotWrapper>& iBotWrapper);
  ~TerrainMap();

  // view id and depth map channel to listen on
  void setInfo(const int64_t iViewId, const std::string& iMapChannel);
  int64_t getViewId() const;
  std::string getMapChannel() const;

  // plane to use when overriding heights and normals
  // and when filling under the robot
  void setFillPlane(const Eigen::Vector4d& iPlane);

  // whether to use the foot pose as the fill plane
  void useFootPose(const bool iVal);

  // whether to override all actual heights with fill plane
  void overrideHeights(const bool iVal);

  // how to compute all actual normals
  void setNormalMethod(const NormalMethod iMethod);

  // whether to fill in missing values in the height map
  void shouldFillMissing(const bool iVal);

  // whether to re-publish processed terrain map
  void shouldPublishMap(const bool iVal, const std::string& iChannel="",
                        const int64_t iViewId=9999);

  // how to compute normals where there is data
  void setNormalRadius(const double iRadius);
  double getNormalRadius() const;

  // start and stop listening to new maps and other messages
  bool startListening();
  bool stopListening();


  // send a height map request to the maps server
  bool sendTimeRequest(const Eigen::Vector3d& iBoxMin,
                       const Eigen::Vector3d& iBoxMax,
                       const double iResolutionMeters,
                       const double iTimeWindowSeconds,
                       const double iFrequencyHz);
  bool sendSweepRequest(const Eigen::Vector3d& iBoxMin,
                        const Eigen::Vector3d& iBoxMax,
                        const double iResolutionMeters,
                        const double iFrequencyHz);


  // get internal depth image data
  std::shared_ptr<TerrainData> getData() const;

  // get height and normal at a given (x,y) location
  template <typename T>
  bool getHeightAndNormal(const T iX, const T iY,
                          T& oHeight, Eigen::Matrix<T,3,1>& oNormal) const;
  
private:
  struct Helper;
  std::shared_ptr<Helper> mHelper;
};

}

#endif
