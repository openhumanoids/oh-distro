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
    bool mActive;
    bool mRelativeTime;
    bool mRelativeLocation;
    Type mType;
    float mResolution;
    float mFrequency;
    int64_t mTimeMin;
    int64_t mTimeMax;
    std::vector<Eigen::Vector4f> mClipPlanes;

    Spec();

    bool operator==(const Spec& iSpec) const;
    bool operator!=(const Spec& iSpec) const;
  };

  struct HeightMap {
    int mWidth;
    int mHeight;
    std::vector<float> mData;
    Eigen::Affine3f mTransform;  // heightmap to reference coords

    typedef boost::shared_ptr<HeightMap> Ptr;
  };

  typedef boost::shared_ptr<MapView> Ptr;

public:
  MapView(const Spec& iSpec);
  virtual ~MapView();

  Spec getSpec() const;

  bool set(const maps::PointCloud& iCloud);
  bool set(const maps::Octree& iTree);

  Ptr clone() const;
  Ptr clone(const Spec& iSpec) const;

  maps::PointCloud::Ptr getAsPointCloud() const;
  HeightMap::Ptr getAsHeightMap(const float iResolution,
                                const float iMaxHeight=1e20) const;
  // TODO: can add other representations here if needed, such as meshes

protected:
  Spec mSpec;
  maps::PointCloud::Ptr mCloud;
  mutable boost::mutex mMutex;
};

}

#endif
