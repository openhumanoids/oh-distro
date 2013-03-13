#include "LocalMap.hpp"

#include <pcl/common/transforms.h>

#include <octomap/octomap.h>

#include "PointDataBuffer.hpp"
#include "Utils.hpp"
#include "PointCloudView.hpp"
#include "OctreeView.hpp"
#include "DepthImageView.hpp"

using namespace maps;

LocalMap::SpaceTimeBounds::
SpaceTimeBounds() {
  mTimeMin = mTimeMax = -1;
}

LocalMap::SpaceTimeBounds::
SpaceTimeBounds(const std::vector<Eigen::Vector4f>& iPlanes,
                const int64_t iTimeMin, const int64_t iTimeMax) {
  set(iPlanes, iTimeMin, iTimeMax);
}

void LocalMap::SpaceTimeBounds::
set(const std::vector<Eigen::Vector4f>& iPlanes,
    const int64_t iTimeMin, const int64_t iTimeMax) {
  mPlanes = iPlanes;
  mTimeMin = iTimeMin;
  mTimeMax = iTimeMax;
}




LocalMap::
LocalMap(const LocalMap::Spec& iSpec) {
  mSpec = iSpec;
  mPointData.reset(new PointDataBuffer());
  mPointData->setMaxLength(mSpec.mPointBufferSize);
  mStateId = 0;
}

LocalMap::
~LocalMap() {
}

void LocalMap::
clear() {
  mPointData->clear();
}

int64_t LocalMap::
getId() const {
  return mSpec.mId;
}

int64_t LocalMap::
getStateId() const {
  return mStateId;
}

int LocalMap::
getMaxPointDataBufferSize() const {
  return mPointData->getMaxLength();
}

Eigen::Vector3f LocalMap::
getBoundMin() const {
  return mSpec.mBoundMin;
}

Eigen::Vector3f LocalMap::
getBoundMax() const {
  return mSpec.mBoundMax;
}

std::vector<Eigen::Vector4f> LocalMap::
getBoundPlanes() const {
  return Utils::planesFromBox(mSpec.mBoundMin, mSpec.mBoundMax);
}

LocalMap::Spec LocalMap::
getSpec() const {
  return mSpec;
}

void LocalMap::
setActive(const bool iVal) {
  mSpec.mActive = iVal;
}

bool LocalMap::isActive() const {
  return mSpec.mActive;
}

bool LocalMap::
addData(const maps::PointSet& iPointSet) {
  // if data is frozen, do not allow new points to be added
  if (!mSpec.mActive) {
    return false;
  }

  // transform points to reference coords
  Eigen::Affine3f xformToReference = Utils::getPose(*iPointSet.mCloud);
  maps::PointCloud refCloud;
  pcl::transformPointCloud(*iPointSet.mCloud, refCloud, xformToReference);

  // set up new point cloud
  maps::PointCloud::Ptr outCloud(new maps::PointCloud());
  outCloud->is_dense = false;

  const float minThresh2 = iPointSet.mMinRange*iPointSet.mMinRange;
  const float maxThresh2 = iPointSet.mMaxRange*iPointSet.mMaxRange;

  // loop over all points
  Eigen::Vector3f refOrigin = xformToReference.translation();
  for (int i = 0; i < refCloud.size(); ++i) {

    // check to see if the point is too close to the sensor
    Eigen::Vector3f refPt(refCloud[i].x, refCloud[i].y, refCloud[i].z);
    if ((refPt - refOrigin).squaredNorm() < minThresh2) {
      continue;
    }

    // clip lidar ray against bounds
    Eigen::Vector3f p1, p2;
    float t1, t2;
    if (!Utils::clipRay(refOrigin, refPt, mSpec.mBoundMin, mSpec.mBoundMax,
                        p1, p2, t1, t2)) {
      continue;
    }

    if ((refPt-refOrigin).squaredNorm() > maxThresh2) {
      continue;
    }

    // check to see whether the point is within bounds
    /* TODO: need? we probably want the points outside the bound
       if their rays intersect the bound, to construct free space properly
    if ((refPt[0] < mSpec.mBoundMin[0]) || (refPt[0] > mSpec.mBoundMax[0]) ||
        (refPt[1] < mSpec.mBoundMin[1]) || (refPt[1] > mSpec.mBoundMax[1]) ||
        (refPt[2] < mSpec.mBoundMin[2]) || (refPt[2] > mSpec.mBoundMax[2])) {
      continue;
    }
    */

    // add point to internal point set
    outCloud->push_back(refCloud[i]);
  }
  
  // if any points survived, add them to buffer and update state count
  if (outCloud->size() > 0) {
    maps::PointSet pointSet = iPointSet;
    pointSet.mCloud = outCloud;
    mPointData->add(pointSet);
    ++mStateId;
  }

  return true;
}

const boost::shared_ptr<PointDataBuffer> LocalMap::
getPointData() const {
  return mPointData;
}

PointCloudView::Ptr LocalMap::
getAsPointCloud(const float iResolution,
                const SpaceTimeBounds& iBounds) const {
  // interpret time bounds
  int64_t timeMin(iBounds.mTimeMin), timeMax(iBounds.mTimeMax);

  // grab point cloud and crop to space-time bounds
  maps::PointCloud::Ptr cloud = mPointData->getAsCloud(timeMin, timeMax);
  Utils::crop(*cloud, *cloud, mSpec.mBoundMin, mSpec.mBoundMax);
  Utils::crop(*cloud, *cloud, iBounds.mPlanes);

  PointCloudView::Ptr view(new PointCloudView());
  view->setResolution(iResolution);
  view->set(cloud);
  return view;
}

OctreeView::Ptr LocalMap::
getAsOctree(const float iResolution, const bool iTraceRays,
            const Eigen::Vector3f& iOrigin,
            const SpaceTimeBounds& iBounds) const {
  int64_t timeMin(iBounds.mTimeMin), timeMax(iBounds.mTimeMax);
  OctreeView::Ptr view(new OctreeView());
  view->setTransform(Eigen::Projective3f(Eigen::Translation3f(-iOrigin)));
  view->setResolution(iResolution);

  if (!iTraceRays) {
    maps::PointCloud::Ptr cloud = mPointData->getAsCloud(timeMin, timeMax);
    Utils::crop(*cloud, *cloud, mSpec.mBoundMin, mSpec.mBoundMax);
    Utils::crop(*cloud, *cloud, iBounds.mPlanes);
    view->set(cloud);
  }

  else {
    std::vector<maps::PointSet> pointSets = mPointData->get(timeMin, timeMax);
    for (int i = 0; i < pointSets.size(); ++i) {
      maps::PointCloud::Ptr inCloud = pointSets[i].mCloud;
      maps::PointCloud::Ptr outCloud(new maps::PointCloud());
      Utils::crop(*inCloud, *outCloud, mSpec.mBoundMin, mSpec.mBoundMax);
      Utils::crop(*outCloud, *outCloud, iBounds.mPlanes);
      octomap::Pointcloud octCloud;    
      for (int j = 0; j < outCloud->points.size(); ++j) {
        octomap::point3d pt(outCloud->points[j].x - iOrigin(0),
                            outCloud->points[j].y - iOrigin(1),
                            outCloud->points[j].z - iOrigin(2));
        octCloud.push_back(pt);
      }
      octomap::point3d octOrigin(-iOrigin[0], -iOrigin[1], -iOrigin[2]);
      view->getOctree()->insertScan(octCloud, octOrigin);
    }
    view->getOctree()->prune();
  }

  return view;
}

DepthImageView::Ptr LocalMap::
getAsDepthImage(const int iWidth, const int iHeight,
                const Eigen::Projective3f& iProjector,
                const SpaceTimeBounds& iBounds) const {
  DepthImageView::Ptr view(new DepthImageView());
  view->setSize(iWidth, iHeight);
  PointCloudView::Ptr cloudView = getAsPointCloud(0, iBounds);
  view->setSize(iWidth, iHeight);
  view->setTransform(iProjector);
  view->set(cloudView->getPointCloud());
  return view;
}
