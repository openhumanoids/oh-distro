#ifndef _terrain_map_TerrainMap_hpp_
#define _terrain_map_TerrainMap_hpp_

#include <memory>
#include <string>
#include <Eigen/Geometry>

namespace maps {
  class BotWrapper;
}

namespace terrainmap {

class TerrainMap {
public:
  struct TerrainData {
    std::vector<float> mHeights;
    int mWidth;
    int mHeight;
    Eigen::Projective3d mTransform;
  };

public:
  TerrainMap(const std::shared_ptr<maps::BotWrapper>& iBotWrapper);
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

  // whether to override all actual normals with fill plane
  void overrideNormals(const bool iVal);

  // whether to fill in missing values in the height map
  void shouldFillMissing(const bool iVal);

  // start and stop listening to new maps and other messages
  bool startListening();
  bool stopListening();



  // get internal depth image data
  std::shared_ptr<TerrainData> getData() const;

  // get height and normal at a given (x,y) location
  bool getHeightAndNormal(const double iX, const double iY,
                          double& oHeight, Eigen::Vector3d& oNormal) const;
  
private:
  struct Helper;
  std::shared_ptr<Helper> mHelper;
};

}

#endif
