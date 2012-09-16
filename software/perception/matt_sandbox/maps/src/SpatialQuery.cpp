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

  if (mCloud->size() == 0) {
    mNeedsUpdate = false;
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

  /* TODO: algorithm sketch:
     find closest cloud point p to query point q
     find point set s within radius r of p
     compute local surface patch using s (mesh? nurbs? other?)
     find closest point c to surface, return as oPoint
     find surface normal at c, return as oNormal
  */

  std::vector<int> indices;
  std::vector<float> distances;

  // find closest point in cloud to query point
  mSearchTree.nearestKSearch(PointType(iPoint[0], iPoint[1], iPoint[2]), 1,
                             indices, distances);
  if (indices.size() == 0) {
    return false;
  }
  PointType pt = mCloud->points[indices[0]];

  // find points in local neighborhood of closest point
  mSearchTree.radiusSearch(pt, mNormalComputationRadius, indices, distances);

  // fit surface to neighborhood points  
  if (indices.size() < 3) {
    return false;
  }
  // TODO: do fit
  pcl::NormalEstimation<PointType,pcl::Normal> est;
  Eigen::Vector4f plane;
  float curvature;
  est.computePointNormal(*mCloud, indices, plane, curvature);

  // find closest point on surface to original query point
  // TODO

  // estimate normal at closest point
  // TODO

  oPoint = Eigen::Vector3d(pt.x, pt.y, pt.z);
  //  oNormal = Eigen::Vector3d(plane.x, plane.y, plane.z);
  return true;
}
