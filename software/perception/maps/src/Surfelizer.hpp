#ifndef _maps_Surfelizer_hpp_
#define _maps_Surfelizer_hpp_

#include "Types.hpp"

namespace maps {

class Surfelizer {
public:
  enum SizeMethod {
    SizeMethodFixed,
    SizeMethodRange,
    SizeMethodAngles,
    SizeMethodNeighbors,
  };

  struct Surfel {
    Eigen::Vector3f mCenter;
    Eigen::Matrix3f mOrientation;  // columns: v1,v2,normal
    Eigen::Vector2f mSize;         // size along v1 and v2
  };

public:
  Surfelizer();
  ~Surfelizer();

  void setScanRadius(const int iRadius);
  void setPointRadius(const int iRadius);
  void setSizeMethod(const SizeMethod iMethod);
  void setNominalSize(const float iSize);
  void setDecimation(const int iDecimation);

  std::vector<Surfel> addScan(const maps::PointSet& iScan);

protected:
  struct Helper;
  std::shared_ptr<Helper> mHelper;
};

}

#endif
