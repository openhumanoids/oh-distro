#ifndef _maps_Utils_hpp_
#define _maps_Utils_hpp_

#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include <vector>

namespace maps {

class Utils {

public:

  template<typename PointType>
  static Eigen::Isometry3f
  getPose(const pcl::PointCloud<PointType>& iCloud) {
    Eigen::Isometry3f xform = Eigen::Isometry3f::Identity();
    xform.linear() = iCloud.sensor_orientation_.matrix();
    Eigen::Vector4f position = iCloud.sensor_origin_;
    xform.translation() = position.head<3>();
    return xform;
  }

  template<typename PointType>
  static void
  setPose(const Eigen::Isometry3f iPose, pcl::PointCloud<PointType>& oCloud) {
    for (int i = 0; i < 3; ++i) {
      oCloud.sensor_origin_[i] = iPose.translation()[i];
    }
    oCloud.sensor_origin_[3] = 1;
    oCloud.sensor_orientation_ = iPose.rotation();
  }

  template<typename PointType1, typename PointType2>
  static void copyPose(const pcl::PointCloud<PointType1>& iCloud,
                       pcl::PointCloud<PointType2>& oCloud) {
    oCloud.sensor_origin_ = iCloud.sensor_origin_;
    oCloud.sensor_orientation_ = iCloud.sensor_orientation_;
  }

  template<typename PointType>
  static bool crop(const pcl::PointCloud<PointType>& iCloud,
                   pcl::PointCloud<PointType>& oCloud,
                   const std::vector<Eigen::Vector4f>& iPlanes) {
    std::vector<int> survivors;
    if (!crop(iCloud, survivors, iPlanes)) {
      return false;
    }
    oCloud.resize(iCloud.size());
    for (int i = 0; i < survivors.size(); ++i) {
      oCloud[i] = iCloud[survivors[i]];
    }
    oCloud.resize(survivors.size());
    oCloud.width = oCloud.size();
    oCloud.height = 1;
    oCloud.is_dense = false;
    copyPose(iCloud, oCloud);
    return true;
  }

  template<typename PointType>
  static bool crop(const pcl::PointCloud<PointType>& iCloud,
                   std::vector<int>& oSurvivors,
                   const std::vector<Eigen::Vector4f>& iPlanes) {
    oSurvivors.clear();
    oSurvivors.reserve(iCloud.size());
    for (int i = 0; i < iCloud.size(); ++i) {
      PointType pt = iCloud[i];
      bool survived = true;
      for (int j = 0; j < iPlanes.size(); ++j) {
        Eigen::Vector4f plane = iPlanes[j].cast<float>();
        if (plane[0]*pt.x + plane[1]*pt.y + plane[2]*pt.z < -plane[3]) {
          survived = false;
          continue;
        }
      }
      if (survived) {
        oSurvivors.push_back(i);
      }
    }
    return true;
  }


  template<typename PointType>
  static bool crop(const pcl::PointCloud<PointType>& iCloud,
                   pcl::PointCloud<PointType>& oCloud,
                   const Eigen::Vector3f& iMin, const Eigen::Vector3f& iMax) {
    std::vector<int> survivors;
    if (!crop(iCloud, survivors, iMin, iMax)) {
      return false;
    }
    oCloud.resize(iCloud.size());
    for (int i = 0; i < survivors.size(); ++i) {
      oCloud[i] = iCloud[survivors[i]];
    }
    oCloud.resize(survivors.size());
    oCloud.width = oCloud.size();
    oCloud.height = 1;
    oCloud.is_dense = false;
    copyPose(iCloud, oCloud);
    return true;
  }

  template<typename PointType>
  static bool crop(const pcl::PointCloud<PointType>& iCloud,
                   std::vector<int>& oSurvivors,
                   const Eigen::Vector3f& iMin, const Eigen::Vector3f& iMax) {
    oSurvivors.clear();
    oSurvivors.reserve(iCloud.size());
    for (int i = 0; i < iCloud.size(); ++i) {
      PointType pt = iCloud[i];
      if ((pt.x >= iMin[0]) && (pt.x <= iMax[0]) &&
          (pt.y >= iMin[1]) && (pt.y <= iMax[1]) &&
          (pt.z >= iMin[2]) && (pt.z <= iMax[2])) {
        oSurvivors.push_back(i);
      }
    }
    return true;
  }


  static bool clipRay(const Eigen::Vector3f& iOrigin,
                      const Eigen::Vector3f& iEndPoint,
                      const Eigen::Vector3f& iBoxMin,
                      const Eigen::Vector3f& iBoxMax,
                      Eigen::Vector3f& oOrigin,
                      Eigen::Vector3f& oEndPoint,
                      float& oMinT, float& oMaxT);

  static bool clipRay(const Eigen::Vector3f& iOrigin,
                      const Eigen::Vector3f& iEndPoint,
                      const std::vector<Eigen::Vector4f>& iPlanes,
                      Eigen::Vector3f& oOrigin,
                      Eigen::Vector3f& oEndPoint,
                      float& oMinT, float& oMaxT);

  static std::vector<Eigen::Vector4f>
  planesFromBox(const Eigen::Vector3f& iBoundMin,
                const Eigen::Vector3f& iBoundMax);

  static bool polyhedronFromPlanes(const std::vector<Eigen::Vector4f>& iPlanes,
                                   std::vector<Eigen::Vector3f>& oVertices,
                                   std::vector<std::vector<int> >& oFaces,
                                   const double iTol=1e-5);

  static std::vector<Eigen::Vector4f>
  planesFromPolyhedron(const std::vector<Eigen::Vector3f>& iVertices,
                       const std::vector<std::vector<int> >& iFaces);

  static bool isOrthographic(const Eigen::Matrix4f& iMatrix);
  static bool composeViewMatrix(Eigen::Projective3f& oMatrix,
                                const Eigen::Matrix3f& iCalib,
                                const Eigen::Isometry3f& iPose,
                                const bool iIsOrthographic);
  static bool factorViewMatrix(const Eigen::Projective3f& iMatrix,
                               Eigen::Matrix3f& oCalib,
                               Eigen::Isometry3f& oPose,
                               bool& oIsOrthographic);

  static uint64_t rand64();
};

}

#endif
