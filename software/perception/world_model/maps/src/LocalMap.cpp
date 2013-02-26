#include "LocalMap.hpp"

#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <octomap/octomap.h>

#include "PointDataBuffer.hpp"
#include "Utils.hpp"

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
  mOctree.mTree.reset(new octomap::OcTree(mSpec.mResolution));
  mOctree.mTransform = Eigen::Isometry3f::Identity();
  mOctree.mTransform.translation() = 0.5f*(mSpec.mBoundMax + mSpec.mBoundMin);
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

  // transform points to octree coords
  Eigen::Affine3f xformToOctree = mOctree.mTransform*xformToReference;
  maps::PointCloud octCloud;
  pcl::transformPointCloud(*iPointSet.mCloud, octCloud, xformToOctree);

  Eigen::Affine3f referenceToOctree = xformToOctree*xformToReference.inverse();

  // set up new point cloud
  maps::PointCloud::Ptr outCloud(new maps::PointCloud());
  outCloud->is_dense = false;

  // loop over all points
  Eigen::Vector3f refOrigin = xformToReference.translation();
  for (int i = 0; i < refCloud.size(); ++i) {

    // check to see if the point is too close to the sensor
    Eigen::Vector3f refPt(refCloud[i].x, refCloud[i].y, refCloud[i].z);
    if (refPt.squaredNorm() < 1e-2*1e-2) {
      continue;
    }

    // clip lidar ray against octree bounds
    // TODO: should octree bounds be separate from other bounds?
    // TODO: do we even need the ref bounds?
    Eigen::Vector3f p1, p2;
    float t1, t2;
    if (!Utils::clipRay(refOrigin, refPt, mSpec.mBoundMin, mSpec.mBoundMax,
                        p1, p2, t1, t2)) {
      continue;
    }

    /* TODO: not using master octree at the moment
    // add ray to octree
    p1 = referenceToOctree*p1;
    p2 = referenceToOctree*p2;
    octomap::point3d octPt1(p1[0], p1[1], p1[2]), octPt2(p2[0], p2[1], p2[2]);
    mOctree.mTree->insertRay(octPt1, octPt2, -1, true);

    // remove return if it is on the boundary
    if (t2 < 1.0f) {
      mOctree.mTree->deleteNode(octPt2);
    }
    */

    float range2 = (refPt-refOrigin).squaredNorm();
    float thresh2 = iPointSet.mMaxRange*iPointSet.mMaxRange;
    if (range2 >= thresh2) {
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
    mOctree.mTree->updateInnerOccupancy();
    ++mStateId;
  }

  return true;
}

const boost::shared_ptr<PointDataBuffer> LocalMap::
getPointData() const {
  return mPointData;
}

maps::PointCloud::Ptr LocalMap::
getAsPointCloud(const float iResolution,
                const SpaceTimeBounds& iBounds) const {
  // interpret time bounds
  int64_t timeMin(iBounds.mTimeMin), timeMax(iBounds.mTimeMax);

  // grab point cloud and crop to space-time bounds
  maps::PointCloud::Ptr cloud = mPointData->getAsCloud(timeMin, timeMax);
  Utils::crop(*cloud, *cloud, mSpec.mBoundMin, mSpec.mBoundMax);
  Utils::crop(*cloud, *cloud, iBounds.mPlanes);

  maps::PointCloud::Ptr cloudFinal(new maps::PointCloud());
  if (iResolution > 0) {
    pcl::VoxelGrid<maps::PointCloud::PointType> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(iResolution,iResolution,iResolution);
    grid.filter(*cloudFinal);
  }
  else {
    cloudFinal = cloud;
  }

  return cloudFinal;
}

maps::Octree LocalMap::
getAsOctree(const float iResolution, const bool iTraceRays,
            const Eigen::Vector3f& iOrigin,
            const SpaceTimeBounds& iBounds) const {
  int64_t timeMin(iBounds.mTimeMin), timeMax(iBounds.mTimeMax);
  Octree oct;
  oct.mTransform = Eigen::Isometry3f::Identity();
  oct.mTransform.translation() = iOrigin;
  oct.mTree.reset(new octomap::OcTree(iResolution));
  std::vector<maps::PointSet> pointSets = mPointData->get(timeMin, timeMax);
  for (int i = 0; i < pointSets.size(); ++i) {
    maps::PointCloud::Ptr inCloud = pointSets[i].mCloud;
    maps::PointCloud::Ptr outCloud(new maps::PointCloud());
    Utils::crop(*inCloud, *outCloud, mSpec.mBoundMin, mSpec.mBoundMax);
    Utils::crop(*outCloud, *outCloud, iBounds.mPlanes);
    Eigen::Vector3f origin = oct.mTransform.translation();
    if (iTraceRays) {
      octomap::Pointcloud octCloud;    
      for (int j = 0; j < outCloud->points.size(); ++j) {
        octomap::point3d pt(outCloud->points[j].x - origin(0),
                            outCloud->points[j].y - origin(1),
                            outCloud->points[j].z - origin(2));
        octCloud.push_back(pt);
      }
      octomap::point3d octOrigin(-origin[0], -origin[1], -origin[2]);
      oct.mTree->insertScan(octCloud, octOrigin);
    }
    else {
      for (int j = 0; j < outCloud->points.size(); ++j) {
        octomap::point3d pt(outCloud->points[j].x - origin(0),
                            outCloud->points[j].y - origin(1),
                            outCloud->points[j].z - origin(2));
        oct.mTree->updateNode(pt, true);
      }
    }
  }
  oct.mTree->prune();
  return oct;
}

maps::RangeImage LocalMap::
getAsRangeImage(const int iWidth, const int iHeight,
                const Eigen::Isometry3f& iPose,
                const Eigen::Matrix4f& iProjector,
                const SpaceTimeBounds& iBounds) const {
  // TODO: can we do orthographic projection?
  maps::PointCloud::Ptr cloud = getAsPointCloud(0, iBounds);
  std::cout << "Cloud has " << cloud->size() << " points." << std::endl;
  maps::RangeImage rangeImage;
  pcl::RangeImagePlanar::Ptr img(new pcl::RangeImagePlanar());
  img->createFromPointCloudWithFixedSize
    (*cloud, iWidth, iHeight, iProjector(0,2), iProjector(1,2),
     iProjector(0,0), iProjector(1,1), iPose);
  rangeImage.mImage = img;
  return rangeImage;
}
