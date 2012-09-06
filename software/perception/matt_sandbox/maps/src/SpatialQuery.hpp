#ifndef _SpatialQuery_hpp_
#define _SpatialQuery_hpp_

#include <pcl/search/kdtree.h>

class SpatialQuery {
protected:
  typedef pcl::PointXYZ PointType;

public:
  SpatialQuery();

  void clear();

  void setNormalComputationRadius(const double iRadius);
  void add(pcl::PointCloud<PointType>::Ptr& iCloud);

  void populateStructures();

  // single point
  bool getClosest(const Eigen::Vector3d& iPoint,
                  Eigen::Vector3d& oPoint);

  // single normal
  bool getClosest(const Eigen::Vector3d& iPoint,
                  Eigen::Vector3d& oPoint, Eigen::Vector3d& oNormal);


protected:
  double mNormalComputationRadius;

  bool mNeedsUpdate;
  pcl::PointCloud<PointType>::Ptr mCloud;
  pcl::search::KdTree<PointType> mSearchTree;
};


#endif
