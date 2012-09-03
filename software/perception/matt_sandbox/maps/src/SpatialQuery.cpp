#include "SpatialQuery.hpp"

#include <pcl/features/normal_3d.h>

SpatialQuery::
SpatialQuery() {
}

void SpatialQuery::
clear() {
  mCloud.reset(new pcl::PointCloud<PointType>());
  mNeedsUpdate = true;
  populateStructures();
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

  // NOTE: can also use other flavor which computes local surface properties
  pcl::NormalEstimation<PointType, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(mCloud);
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
  normalEstimator.setSearchMethod(tree);  // TODO: redundant?
  normalEstimator.setRadiusSearch(0.1);
  mNormals.reset(new pcl::PointCloud<pcl::Normal>());
  normalEstimator.compute(*mNormals);
  mNeedsUpdate = false;
}

bool SpatialQuery::
getClosest(const Eigen::Vector3d& iPoint,
           Eigen::Vector3d& oPoint) {
  std::vector<int> indices;
  std::vector<float> distances;
  mSearchTree.nearestKSearch(PointType(iPoint[0], iPoint[1], iPoint[2]), 1,
                             indices, distances);
  if (indices.size() == 0) {
    return false;
  }

  PointType pt = mCloud->points[indices[0]];
  oPoint = Eigen::Vector3d(pt.x, pt.y, pt.z);
  return true;
}

bool SpatialQuery::
getClosest(const Eigen::Vector3d& iPoint,
           Eigen::Vector3d& oPoint, Eigen::Vector3d& oNormal) {
  std::vector<int> indices;
  std::vector<float> distances;
  mSearchTree.nearestKSearch(PointType(iPoint[0], iPoint[1], iPoint[2]), 1,
                             indices, distances);
  if (indices.size() == 0) {
    return false;
  }

  PointType pt = mCloud->points[indices[0]];
  pcl::Normal normal = mNormals->points[indices[0]];
  oPoint = Eigen::Vector3d(pt.x, pt.y, pt.z);
  oNormal = Eigen::Vector3d(normal.normal_x, normal.normal_y, normal.normal_z);
  return true;
}
