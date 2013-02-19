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

  class Filter {
  public:
    virtual bool operate(const maps::PointCloud& iCloud,
                         maps::PointCloud& oCloud) const = 0;
    typedef boost::shared_ptr<Filter> Ptr;
  };

  struct HeightMap {
    int mWidth;
    int mHeight;
    std::vector<float> mData;
    Eigen::Affine3f mTransform;  // heightmap to reference coords
    typedef boost::shared_ptr<HeightMap> Ptr;
  };

  struct TriangleMesh {
    std::vector<Eigen::Vector3f> mVertices;
    std::vector<Eigen::Vector3i> mFaces;
    typedef boost::shared_ptr<TriangleMesh> Ptr;
  };

  typedef boost::shared_ptr<MapView> Ptr;

public:
  MapView(const Spec& iSpec);
  virtual ~MapView();

  Spec getSpec() const;

  bool set(const maps::PointCloud& iCloud);
  bool set(const maps::Octree& iTree);

  void setUpdateTime(const int64_t iTime);
  int64_t getUpdateTime() const;

  Ptr clone() const;
  Ptr clone(const Spec& iSpec) const;

  maps::PointCloud::Ptr
  getAsPointCloud(const Filter::Ptr& iFilter=Filter::Ptr()) const;

  HeightMap::Ptr
  getAsHeightMap(const float iResolution, const float iMaxHeight=1e20,
                 const Filter::Ptr& iFilter=Filter::Ptr()) const;

  TriangleMesh::Ptr getAsMesh(const Filter::Ptr& iFilter=Filter::Ptr()) const;

protected:
  Spec mSpec;
  int64_t mUpdateTime;
  maps::PointCloud::Ptr mCloud;
  mutable boost::mutex mMutex;
};

}

#endif
