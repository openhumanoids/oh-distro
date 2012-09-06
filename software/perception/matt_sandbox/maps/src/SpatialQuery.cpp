#include "SpatialQuery.hpp"

#include <pcl/features/normal_3d.h>

SpatialQuery::
SpatialQuery() {
  clear();
  setNormalComputationRadius(0.1);
}

void SpatialQuery::
clear() {
  mCloud.reset(new pcl::PointCloud<PointType>());
  mNeedsUpdate = true;
  populateStructures();
}

void SpatialQuery::
setNormalComputationRadius(const double iRadius) {
  mNormalComputationRadius = iRadius;
  mNeedsUpdate = true;
}

void SpatialQuery::
add(pcl::PointCloud<PointType>::Ptr& iCloud) {
  *mCloud += *iCloud;
  mNeedsUpdate = true;
}

void SpatialQuery::
populateStructures() {
  if (!mNeedsUpdate) {
    return;
  }

  mSearchTree.setInputCloud(mCloud);
  mNeedsUpdate = false;
}

bool SpatialQuery::
getClosest(const Eigen::Vector3d& iPoint, Eigen::Vector3d& oPoint) {
  Eigen::Vector3d dummyNormal;
  return getClosest(iPoint, oPoint, dummyNormal);
}

bool SpatialQuery::
getClosest(const Eigen::Vector3d& iPoint,
           Eigen::Vector3d& oPoint, Eigen::Vector3d& oNormal) {
  std::vector<int> indices;
  std::vector<float> distances;

  // first find closest point in cloud to query point
  mSearchTree.nearestKSearch(PointType(iPoint[0], iPoint[1], iPoint[2]), 1,
                             indices, distances);
  if (indices.size() == 0) {
    return false;
  }
  PointType pt = mCloud->points[indices[0]];

  // next find points in local neighborhood of closest point
  // and fit a surface
  mSearchTree.radiusSearch(pt, mNormalComputationRadius, indices, distances);
  if (indices.size() < 3) {
    return false;
  }
  // TODO: fit surface, probably quadratic

  // finally, find closest point on surface to original query point
  // TODO

  oPoint = Eigen::Vector3d(pt.x, pt.y, pt.z);
  return true;
}
