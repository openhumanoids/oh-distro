#ifndef _SpatialQuery_hpp_
#define _SpatialQuery_hpp_

#include <pcl/search/kdtree.h>

class LocalMap;

class SpatialQuery {
protected:
  typedef pcl::PointXYZ PointType;

public:
  enum LocationOccupancy {
    LocationUnobserved,
    LocationOccupied,
    LocationFree
  };

public:
  SpatialQuery();

  void clear();

  void setNormalComputationRadius(const double iRadius);

  void setMap(const boost::shared_ptr<LocalMap>& iMap);

  void populateStructures();

  // get point closest to input query point
  bool getClosest(const Eigen::Vector3d& iPoint,
                  Eigen::Vector3d& oPoint);

  // get point+normal closest to query point
  bool getClosest(const Eigen::Vector3d& iPoint,
                  Eigen::Vector3d& oPoint, Eigen::Vector3d& oNormal);

  // determine whether a particular point has been observed
  bool isObserved(const Eigen::Vector3d& iPoint);

  // determine status of a particular point (unobserved, occupied, or free)
  LocationOccupancy getOccupancy(const Eigen::Vector3d& iPoint);

protected:
  double mNormalComputationRadius;

  bool mNeedsUpdate;
  boost::shared_ptr<LocalMap> mMap;
  pcl::PointCloud<PointType>::Ptr mCloud;
  pcl::search::KdTree<PointType> mSearchTree;
};


#endif
