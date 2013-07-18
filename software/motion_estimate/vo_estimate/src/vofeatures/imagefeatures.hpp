#ifndef __IMAGEFEATURES_hpp__
#define __IMAGEFEATURES_hpp__


struct ImageFeature
{
  int track_id;
  Eigen::Vector2d uv; ///< unrectified, distorted, orig. coords
  Eigen::Vector2d base_uv; ///< unrectified, distorted, base level. [these seem to be the actual orig. coords]
  Eigen::Vector3d uvd; ///< rectified, undistorted, base level
  Eigen::Vector3d xyz;
  Eigen::Vector4d xyzw;
  uint8_t color[3];

  // @todo what more is needed?
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
