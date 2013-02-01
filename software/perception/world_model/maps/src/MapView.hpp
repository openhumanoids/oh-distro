#ifndef _maps_MapView_hpp_
#define _maps_MapView_hpp_

#include "Types.hpp"
#include <boost/thread/mutex.hpp>

namespace maps {

class MapView {
public:
  struct Spec {
    enum Type {
      TypeOctree,
      TypeCloud
    };

    int64_t mMapId;
    int64_t mViewId;
    Type mType;
    float mResolution;
    float mFrequency;
    int64_t mTimeMin;
    int64_t mTimeMax;
    std::vector<Eigen::Vector4f> mClipPlanes;

    Spec() {
      mMapId = mViewId = 0;
      mType = TypeCloud;
      mResolution = 0;
      mFrequency = 0;
      mTimeMin = mTimeMax = 0;
    }
  };

  typedef boost::shared_ptr<MapView> Ptr;

public:
  MapView(const Spec& iSpec);
  virtual ~MapView();

  Spec getSpec() const;

  bool set(const maps::PointCloud& iCloud);
  bool set(const maps::Octree& iTree);

  Ptr clone() const;

  maps::PointCloud::Ptr getAsPointCloud() const;
  // TODO: can add other representations here if needed, such as meshes

protected:
  Spec mSpec;
  maps::PointCloud::Ptr mCloud;
  mutable boost::mutex mMutex;
};

}

#endif
