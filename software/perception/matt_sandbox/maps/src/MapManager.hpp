#ifndef _MapManager_hpp_
#define _MapManager_hpp_

#include <unordered_map>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

class PointDataBuffer;
class MapChunk;

// TODO: use pointers to point clouds everywhere?

class MapManager {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  struct MapDelta {
    int64_t mCurrentTime;
    int64_t mPreviousTime;
    PointCloud::Ptr mAdded;
    PointCloud::Ptr mRemoved;
  };

protected:
  typedef boost::shared_ptr<MapChunk> MapPtr;
  typedef std::unordered_map<int64_t, MapPtr> MapCollection;


public:
  // constructor/destructor
  MapManager();
  ~MapManager();

  // completely clear the state
  void clear();

  // setters for map properties
  void setMapResolution(const double iResolution);
  void setMapDimensions(const Eigen::Vector3d iDims);
  void setDataBufferLength(const int iLength);

  // create new map and make it the active one
  bool createMap(const Eigen::Isometry3d& iToLocal);

  // switch to use map with given id
  bool useMap(const int64_t iId);

  // get id of current map, or -1 if there is no current map
  int64_t getActiveMapId() const;

  // reset state of current map chunk
  bool clearActiveMap();

  // buffer input points, transform to local frame, add to current map
  bool add(const int64_t iTime, const PointCloud::Ptr& iPoints,
           const Eigen::Isometry3d& iToLocal, const bool iBuffer=true);

  // remove a set of points (in local frame) from current map
  bool removeFromMap(const PointCloud::Ptr& iCloud);

  // fuse all current buffered points to rebuild current map
  bool fuseAll();

  // compute delta between current and previous version of current map
  bool computeDelta(MapDelta& oDelta);

  // get point cloud for entire (current) map
  PointCloud::Ptr getPointCloud() const;

  // make previous version of map same as current for deltas
  bool resetDelta();

  // serialize to bytes
  void serialize(std::vector<char>& oBytes) const;

  // deserialize from bytes
  void deserialize(const std::vector<char>& iBytes);

protected:
  MapPtr mActiveMap;
  MapPtr mActiveMapPrev;
  MapCollection mMaps;
  boost::shared_ptr<PointDataBuffer> mPointDataBuffer;

  double mMapResolution;
  Eigen::Vector3d mMapDimensions;
  int mDataBufferLength;

  int mNextMapId;
};

#endif
