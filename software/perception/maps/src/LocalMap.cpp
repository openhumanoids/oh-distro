#include "LocalMap.hpp"

#include <pcl/common/transforms.h>

#include <octomap/octomap.h>

#include "PointDataBuffer.hpp"
#include "LidarScan.hpp"
#include "Utils.hpp"
#include "PointCloudView.hpp"
#include "OctreeView.hpp"
#include "DepthImageView.hpp"
#include "DepthImage.hpp"
#include "ScanBundleView.hpp"

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


LocalMap::RangeFilter::
RangeFilter() {
  setValidRanges(0, 1e10);
}

void LocalMap::RangeFilter::
setValidRanges(const float iMin, const float iMax) {
  mRangeMin = iMin;
  mRangeMax = iMax;
}

void LocalMap::RangeFilter::
operator()(LidarScan& ioScan) {
  int n = ioScan.getRanges().size();
  for (int i = 0; i < n; ++i) {
    float range = ioScan.range(i);
    if ((range < mRangeMin) || (range > mRangeMax)) {
      ioScan.range(i) = -fabs(range);
    }
  }
}

LocalMap::RangeDiffFilter::
RangeDiffFilter() {
  mDiffMax = 1e10;
  mRangeMax = 0;
}

void LocalMap::RangeDiffFilter::
set(const float iDiffMax, const float iRangeMax) {
  mDiffMax = iDiffMax;
  mRangeMax = iRangeMax;
}

void LocalMap::RangeDiffFilter::
operator()(LidarScan& ioScan) {
  int n = ioScan.getRanges().size();
  for (int i = 1; i < n-1; ++i) {
    float range = fabs(ioScan.range(i));
    float range1 = fabs(ioScan.range(i-1));
    float range2 = fabs(ioScan.range(i+1));
    float diff1 = fabs(range-range1);
    float diff2 = fabs(range-range2);
    if (((diff1 > mDiffMax) || (diff2 > mDiffMax)) && (range < mRangeMax)) {
      ioScan.range(i) = -range;
    }
  }
}

LocalMap::RangeAngleFilter::
RangeAngleFilter() {
  set(30);
}

void LocalMap::RangeAngleFilter::
set(const float iThetaMin) {
  mThetaMin = iThetaMin;
}

void LocalMap::RangeAngleFilter::
operator()(LidarScan& ioScan) {
  const int n = ioScan.getNumRanges();
  const float angleThresh = mThetaMin*M_PI/180;
  for (int i = 1; i < n-1; ++i) {
    if ((ioScan.range(i-1) <= 0) || (ioScan.range(i) <= 0) ||
        (ioScan.range(i+1) <= 0)) continue;
    Eigen::Vector2f p1 = ioScan.getVector(i-1).head<2>();
    Eigen::Vector2f p2 = ioScan.getVector(i).head<2>();
    Eigen::Vector2f p3 = ioScan.getVector(i+1).head<2>();
    Eigen::Vector2f pointDelta1 = (p1-p2).normalized();
    Eigen::Vector2f pointDelta2 = (p3-p2).normalized();
    Eigen::Vector2f ray = p2.normalized();
    float angle1 = std::acos(ray.dot(pointDelta1));
    if (angle1 > M_PI/2) angle1 = M_PI-angle1;
    float angle2 = std::acos(ray.dot(pointDelta2));
    if (angle2 > M_PI/2) angle2 = M_PI-angle2;
    if ((angle1 < angleThresh) && (angle2 < angleThresh)) {
      ioScan.range(i) = -ioScan.range(i);
    }
  }
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
  std::unique_lock<std::mutex> lock(mScanMutex);
  mScanData.clear();
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

void LocalMap::
addFilter(const std::shared_ptr<Filter>& iFilter) {
  mFilters.push_back(iFilter);
}

bool LocalMap::
addData(const maps::LidarScan& iScan) {
  if (!mSpec.mActive) return false;
  LidarScan scan = iScan;
  for (auto filter : mFilters) (*filter)(scan);
  {
    std::unique_lock<std::mutex> lock(mScanMutex);
    mScanData.push_back(LidarScan::Ptr(new LidarScan(scan)));
    while(mScanData.size() > getMaxPointDataBufferSize()) mScanData.pop_front();
  }
  PointSet points;
  scan.get(points);
  return addData(points);
}

bool LocalMap::
addData(const maps::PointSet& iPointSet) {
  // if data is frozen, do not allow new points to be added
  if (!mSpec.mActive) {
    return false;
  }

  // apply filters
  PointSet points = iPointSet;
  for (auto filter : mFilters) (*filter)(points);

  // transform points to reference coords
  Eigen::Affine3f xformToReference = Utils::getPose(*points.mCloud);
  maps::PointCloud refCloud;
  pcl::transformPointCloud(*points.mCloud, refCloud, xformToReference);

  // set up new point cloud
  maps::PointCloud::Ptr outCloud(new maps::PointCloud());
  outCloud->is_dense = false;

  // loop over all points
  Eigen::Vector3f refOrigin = xformToReference.translation();
  for (int i = 0; i < refCloud.size(); ++i) {

    // TODO: can add filters back in if necessary

    // add point to internal point set
    outCloud->push_back(refCloud[i]);
  }
  
  // if any points survived, add them to buffer and update state count
  if (outCloud->size() > 0) {
    maps::PointSet pointSet = points;
    pointSet.mCloud = outCloud;
    mPointData->add(pointSet);
    ++mStateId;
  }

  return true;
}

const std::shared_ptr<PointDataBuffer> LocalMap::
getPointData() const {
  return mPointData;
}

std::deque<LidarScan::Ptr> LocalMap::
getScanData() const {
  std::unique_lock<std::mutex> lock(mScanMutex);
  return mScanData;
}

PointCloudView::Ptr LocalMap::
getAsPointCloud(const float iResolution,
                const SpaceTimeBounds& iBounds) const {
  // interpret time bounds
  int64_t timeMin(iBounds.mTimeMin), timeMax(iBounds.mTimeMax);

  // grab point cloud and crop to space-time bounds
  maps::PointCloud::Ptr cloud = mPointData->getAsCloud(timeMin, timeMax);
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
    Utils::crop(*cloud, *cloud, iBounds.mPlanes);
    view->set(cloud);
  }

  else {
    std::vector<maps::PointSet> pointSets = mPointData->get(timeMin, timeMax);
    for (int i = 0; i < pointSets.size(); ++i) {
      maps::PointCloud::Ptr inCloud = pointSets[i].mCloud;
      maps::PointCloud::Ptr outCloud(new maps::PointCloud());
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
  return getAsDepthImage(iWidth, iHeight, iProjector,
                         DepthImage::AccumulationMethodClosest, iBounds);
}

DepthImageView::Ptr LocalMap::
getAsDepthImage(const int iWidth, const int iHeight,
                const Eigen::Projective3f& iProjector,
                const int iAccumMethod,
                const SpaceTimeBounds& iBounds) const {
  DepthImageView::Ptr view(new DepthImageView());
  view->setSize(iWidth, iHeight);
  if (iAccumMethod >= 0) {
    DepthImage::AccumulationMethod method =
      (DepthImage::AccumulationMethod)iAccumMethod;
    view->getDepthImage()->setAccumulationMethod(method);
  }
  PointCloudView::Ptr cloudView = getAsPointCloud(0, iBounds);
  view->setSize(iWidth, iHeight);
  view->setTransform(iProjector);
  view->set(cloudView->getPointCloud());
  return view;
}

ScanBundleView::Ptr LocalMap::
getAsScanBundle(const SpaceTimeBounds& iBounds) const {
  // TODO: can have dedicated scan buffer object
  ScanBundleView::Ptr view(new ScanBundleView());
  std::deque<std::shared_ptr<LidarScan>> allScans;
  {
    std::unique_lock<std::mutex> lock(mScanMutex);
    allScans = mScanData;
  }
  std::vector<LidarScan::Ptr> scans;
  scans.reserve(allScans.size());
  for (auto scan : allScans) {
    int64_t t = scan->getTimestamp();
    if ((t < iBounds.mTimeMin) || (t > iBounds.mTimeMax)) continue;
    scans.push_back(scan);
  }
  view->set(scans);
  return view;
}
