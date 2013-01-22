#include "Utils.hpp"

using namespace maps;

// helper functions
template<int Idx, int Sgn>
static inline bool
clip(const Eigen::Vector3f& iOrigin, const Eigen::Vector3f& iDirection,
     const Eigen::Vector3f& iPlanePoint,
     Eigen::Vector3f& oOrigin, Eigen::Vector3f& oEndPoint,
     float& oMinT, float& oMaxT) {
  float planeDist = -Sgn*iPlanePoint[Idx];
  float dist1 = Sgn*oOrigin[Idx] + planeDist;
  float dist2 = Sgn*oEndPoint[Idx] + planeDist;
  if ((dist1 < 0) && (dist2 < 0)) return false;
  if ((dist1 >= 0) && (dist2 >= 0)) return true;
  float t = (Sgn*iOrigin[Idx]+planeDist)/iDirection[Idx];
  if (dist1 >= 0) {
    oMaxT = t;
    oEndPoint = iOrigin + iDirection*t;
  }
  if (dist2 >= 0) {
    oMinT = t;
    oOrigin = iOrigin + iDirection*t;
  }
  return true;
}



bool Utils::
clipRay(const Eigen::Vector3f& iOrigin, const Eigen::Vector3f& iEndPoint,
        const Eigen::Vector3f& iBoxMin, const Eigen::Vector3f& iBoxMax,
        Eigen::Vector3f& oOrigin, Eigen::Vector3f& oEndPoint,
        float& oMinT, float& oMaxT) {

  Eigen::Vector3f dir = iEndPoint - iOrigin;
  oMinT = 0;
  oMaxT = 1;
  oOrigin = iOrigin;
  oEndPoint = iEndPoint;

  float dist1, dist2, denom;

  // n'x = -d
  // x = o + r t
  // -d = n'o + n'r t
  // t = -(d+n'o)/n'r

  // xmin plane:  1,0,0,-xmin
  if (!clip<0,1>(iOrigin, dir, iBoxMin, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // xmax plane: -1,0,0,xmax
  if (!clip<0,-1>(iOrigin, dir, iBoxMax, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // ymin plane:  0,1,0,-ymin
  if (!clip<1,1>(iOrigin, dir, iBoxMin, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // ymax plane:  0,-1,0,ymax
  if (!clip<1,-1>(iOrigin, dir, iBoxMax, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // zmin plane:  0,0,1,-zmin
  if (!clip<2,1>(iOrigin, dir, iBoxMin, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  // zmax plane:  0,0,-1,zmax
  if (!clip<2,-1>(iOrigin, dir, iBoxMax, oOrigin, oEndPoint, oMinT, oMaxT)) {
    return false;
  }

  return true;
}

bool Utils::
clipRay(const Eigen::Vector3f& iOrigin, const Eigen::Vector3f& iEndPoint,
        const std::vector<Eigen::Vector4f>& iPlanes,
        Eigen::Vector3f& oOrigin, Eigen::Vector3f& oEndPoint,
        float& oMinT, float& oMaxT) {
  Eigen::Vector3f dir = iEndPoint - iOrigin;
  oMinT = 0;
  oMaxT = 1;
  oOrigin = iOrigin;
  oEndPoint = iEndPoint;
  for (int i = 0; i < iPlanes.size(); ++i) {
    Eigen::Vector4f plane = iPlanes[i];
    Eigen::Vector3f normal(plane[0], plane[1], plane[2]);
    float dist1 = normal.dot(oOrigin) + plane[3];
    float dist2 = normal.dot(oEndPoint) + plane[3];
    if ((dist1 < 0) && (dist2 < 0)) return false;
    if ((dist1 >= 0) && (dist2 >= 0)) continue;
    float t = (normal.dot(iOrigin) + plane[3]) / normal.dot(dir);
    if (dist1 >= 0) {
      oMaxT = t;
      oEndPoint = iOrigin + dir*t;
    }
    if (dist2 >= 0) {
      oMinT = t;
      oOrigin = iOrigin + dir*t;
    }
  }
  return true;
}

int64_t Utils::
rand64() {
  return (rand() << 31) + rand();
}
