#include "SpatialQuery.hpp"

#include "LocalMap.hpp"
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
setMap(const boost::shared_ptr<LocalMap>& iMap) {
  mMap = iMap;
  mNeedsUpdate = true;
}

void SpatialQuery::
populateStructures() {
  if (!mNeedsUpdate) {
    return;
  }

  if (mMap == NULL) {
    mNeedsUpdate = false;
    return;
  }

  mCloud = mMap->getAsPointCloud();
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

  if (mNeedsUpdate) {
    return false;
    // TODO: could just update here
  }

  if (mCloud->size() < 3) {
    return false;
  }

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
  pcl::NormalEstimation<PointType,pcl::Normal> est;
  Eigen::Vector4f plane;
  float curvature;
  est.computePointNormal(*mCloud, indices, plane, curvature);

  // find closest point on surface to original query point
  // TODO: for now, use closest point cloud point

  // estimate normal at closest point
  // TODO: for now, use the plane normal

  oPoint = Eigen::Vector3d(pt.x, pt.y, pt.z);
  oNormal = Eigen::Vector3d(plane[0], plane[1], plane[2]);
  oNormal.normalize();
  return true;
}

bool SpatialQuery::
isObserved(const Eigen::Vector3d& iPoint) {
  return (getOccupancy(iPoint) != LocationUnobserved);
}

SpatialQuery::LocationOccupancy SpatialQuery::
getOccupancy(const Eigen::Vector3d& iPoint) {
  //octomap::point3d pt(iPoint[0], iPoint[1], iPoint[2]);
  /* TODO: need to expose these methods
  octomap::OcTreeNode* node = mOctree->search(pt);
  if (node == NULL) {
    return LocationUnobserved;
  }
  return (mOctree->isNodeOccupied(node) ? LocationOccupied : LocationFree);
  */
  return LocationUnobserved;  // TODO: temporary
}
