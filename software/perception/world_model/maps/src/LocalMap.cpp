#include "LocalMap.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <octomap/octomap.h>

#include "PointDataBuffer.hpp"
#include "Utils.hpp"

using namespace maps;

LocalMap::SpaceTimeBounds::
SpaceTimeBounds() {
  mMinTime = mMaxTime = -1;
}

LocalMap::SpaceTimeBounds::
SpaceTimeBounds(const std::vector<Eigen::Vector4f>& iPlanes,
                const int64_t iMinTime, const int64_t iMaxTime) {
  set(iPlanes, iMinTime, iMaxTime);
}

void LocalMap::SpaceTimeBounds::
set(const std::vector<Eigen::Vector4f>& iPlanes,
    const int64_t iMinTime, const int64_t iMaxTime) {
  mPlanes = iPlanes;
  mMinTime = iMinTime;
  mMaxTime = iMaxTime;
}




LocalMap::
LocalMap(const LocalMap::Spec& iSpec) {
  mSpec = iSpec;
  mPointData.reset(new PointDataBuffer());
  mPointData->setMaxLength(mSpec.mPointBufferSize);
  mOctree.mTree.reset(new octomap::OcTree(mSpec.mOctreeResolution));
  mOctree.mTransform = mSpec.mOctreeTransform;
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
  std::vector<Eigen::Vector4f> planes(6);
  planes[0] = Eigen::Vector4f( 1, 0, 0, -mSpec.mBoundMin[0]);
  planes[1] = Eigen::Vector4f(-1, 0, 0,  mSpec.mBoundMax[0]);
  planes[2] = Eigen::Vector4f( 0, 1, 0, -mSpec.mBoundMin[1]);
  planes[3] = Eigen::Vector4f( 0,-1, 0,  mSpec.mBoundMax[1]);
  planes[4] = Eigen::Vector4f( 0, 0, 1, -mSpec.mBoundMin[2]);
  planes[5] = Eigen::Vector4f( 0, 0,-1,  mSpec.mBoundMax[2]);
  return planes;
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
  Eigen::Affine3f xformToReference = Utils::getPoseMatrix(*iPointSet.mCloud);
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

    // add ray to octree
    p1 = referenceToOctree*p1;
    p2 = referenceToOctree*p2;
    octomap::point3d octPt1(p1[0], p1[1], p1[2]), octPt2(p2[0], p2[1], p2[2]);
    mOctree.mTree->insertRay(octPt1, octPt2, -1, true);

    // remove return if it is on the boundary
    if (t2 < 1.0f) {
      mOctree.mTree->deleteNode(octPt2);
    }

    // check to see whether the point is within bounds
    if ((refPt[0] < mSpec.mBoundMin[0]) || (refPt[0] > mSpec.mBoundMax[0]) ||
        (refPt[1] < mSpec.mBoundMin[1]) || (refPt[1] > mSpec.mBoundMax[1]) ||
        (refPt[2] < mSpec.mBoundMin[2]) || (refPt[2] > mSpec.mBoundMax[2])) {
      continue;
    }

    // add point to internal point set
    outCloud->push_back(refCloud[i]);
  }
  mOctree.mTree->updateInnerOccupancy();
  
  // add surviving points to buffer
  if (outCloud->size() > 0) {
    maps::PointSet pointSet;
    pointSet.mTimestamp = iPointSet.mTimestamp;
    pointSet.mCloud = outCloud;
    mPointData->add(pointSet);
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
  // grab point cloud and crop to space-time bounds
  maps::PointCloud::Ptr cloud =
    mPointData->getAsCloud(iBounds.mMinTime, iBounds.mMaxTime);
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

LocalMap::Octree LocalMap::
getAsOctree(const float iResolution, const bool iTraceRays,
            const Eigen::Vector3f& iShift,
            const SpaceTimeBounds& iBounds) const {
  //  return mOctree;

  /* TODO: want to copy and crop the actual octree rather than create new
  std::stringstream ss;
  mOctree.mTree->writeBinaryConst(ss);
  ss.flush();
  Octree oct;
  oct.mTree.reset(new octomap::OcTree(mOctree.mTree->getResolution()));
  oct.mTree->readBinary(ss);
  //  mOctree.mTree->deepCopy();
  oct.mTransform = mOctree.mTransform;
  return oct;
  */

  Octree oct;
  oct.mTransform = Eigen::Isometry3f::Identity();
  oct.mTransform(0,3) = iShift(0);
  oct.mTransform(1,3) = iShift(1);
  oct.mTransform(2,3) = iShift(2);
  oct.mTree.reset(new octomap::OcTree(iResolution));
  std::vector<maps::PointSet> pointSets =
    mPointData->get(iBounds.mMinTime, iBounds.mMaxTime);
  for (int i = 0; i < pointSets.size(); ++i) {
    maps::PointCloud::Ptr inCloud = pointSets[i].mCloud;
    maps::PointCloud::Ptr outCloud(new maps::PointCloud());
    Utils::crop(*inCloud, *outCloud, iBounds.mPlanes);
    if (iTraceRays) {
      octomap::Pointcloud octCloud;    
      for (int j = 0; j < outCloud->points.size(); ++j) {
        octomap::point3d pt(outCloud->points[j].x + iShift(0),
                            outCloud->points[j].y + iShift(1),
                            outCloud->points[j].z + iShift(2));
        octCloud.push_back(pt);
      }
      Eigen::Vector3f origin = oct.mTransform.translation();
      octomap::point3d octOrigin(origin[0], origin[1], origin[2]);
      oct.mTree->insertScan(octCloud, octOrigin);
    }
    else {
      for (int j = 0; j < outCloud->points.size(); ++j) {
        octomap::point3d pt(outCloud->points[j].x + iShift(0),
                            outCloud->points[j].y + iShift(1),
                            outCloud->points[j].z + iShift(2));
        oct.mTree->updateNode(pt, true);
      }
    }
  }
  oct.mTree->prune();
  return oct;
}



/*
LocalMap::HeightMap LocalMap::
getAsHeightMap(const int iDownSample,
               const float iMaxHeight) const {
  const float unobservedValue = -std::numeric_limits<float>::max();
  HeightMap heightMap;

  // clamp downsample factor
  int downSample = iDownSample;
  if (downSample < 1) {
    std::cout << "Warning: changing downsample factor from " << downSample <<
      " to 1" << std::endl;
    downSample = 1;
  }

  // determine 2d extents of octree (x,y) in units of cells
  // TODO: could also create a hash map of occupied keys
  // TODO: ... and do everything in a single pass
  Octree::iterator iter;
  int xMin(65536), yMin(65536), zMin(65536);
  int xMax(-65536), yMax(-65536), zMax(-65536);
  for (iter = mOctree->begin_leafs(); iter != mOctree->end_leafs(); ++iter) {
    octomap::OcTreeKey key = iter.getKey();
    bool occupied = mOctree->isNodeOccupied(*iter);
    if (!occupied) {
      continue;
    }
    xMin = std::min(xMin, (int)key[0] >> downSample);
    yMin = std::min(yMin, (int)key[1] >> downSample);
    zMin = std::min(zMin, (int)key[2]);
    xMax = std::max(xMax, (int)key[0] >> downSample);
    yMax = std::max(yMax, (int)key[1] >> downSample);
    zMax = std::max(zMax, (int)key[2]);
  }

  // determine transform from image to local coordinates
  float scale = mOctree->getResolution() * (1 << downSample);
  float offset = mOctree->keyToCoord(0);
  Eigen::Affine3d xform = Eigen::Affine3d::Identity();
  xform(0,0) = xform(1,1) = scale;
  xform(0,3) = offset + xMin*scale;
  xform(1,3) = offset + yMin*scale;
  heightMap.mTransformToLocal = mTransformToLocal*xform;
  heightMap.mMinZ = offset + zMin*mOctree->getResolution();
  heightMap.mMaxZ = offset + zMax*mOctree->getResolution();
  
  // initialize height map data
  heightMap.mWidth = xMax-xMin+1;
  heightMap.mHeight = yMax-yMin+1;
  int totalPixels = heightMap.mWidth * heightMap.mHeight;
  heightMap.mData.resize(totalPixels);
  for (int i = 0; i < heightMap.mData.size(); ++i) {
    // TODO: could use NaN here instead
    heightMap.mData[i] = unobservedValue;
  }

  // add height values into height map
  for (iter = mOctree->begin_leafs(); iter != mOctree->end_leafs(); ++iter) {
    octomap::OcTreeKey key = iter.getKey();
    float z = iter.getZ();
    bool occupied = mOctree->isNodeOccupied(*iter);
    if (!occupied) {
      continue;
    }
    if (z <= iMaxHeight) {
      int xKey = (int)key[0] >> downSample;
      int yKey = (int)key[1] >> downSample;
      int index = (yKey-yMin)*heightMap.mWidth + (xKey-xMin);
      heightMap.mData[index] = std::max(heightMap.mData[index], z);
    }
  }

  return heightMap;
}

LocalMap::DepthMap LocalMap::
getAsDepthMap(const Eigen::Projective3d& iLocalToImage,
              const int iWidth, const int iHeight) const {
  DepthMap depthMap;
  depthMap.mWidth = iWidth;
  depthMap.mHeight = iHeight;
  depthMap.mTransform = iLocalToImage;

  Eigen::Projective3d xformToImage = iLocalToImage*mTransformToLocal;
  Eigen::Projective3d xformFromImage = xformToImage.inverse();
  Eigen::Vector4d bottomRow = xformToImage.matrix().bottomRows<1>();
  Eigen::Vector4d o = xformFromImage.matrix()*bottomRow;
  octomap::point3d origin(o(0)/o(3), o(1)/o(3), o(2)/o(3));
  octomap::point3d direction;
  bool orthographic = (fabs(bottomRow(3)) > fabs(bottomRow(2)));

  depthMap.mData.resize(iWidth*iHeight);
  const float unobservedValue = -std::numeric_limits<float>::max();
  for (int i = 0; i < iHeight; ++i) {
    for (int j = 0; j < iWidth; ++j) {
      int index = i*iWidth + j;
      // TODO: may need to change origin if ortho
      Eigen::Vector4d dir = xformFromImage*Eigen::Vector4d(j,i,1,1);
      octomap::point3d direction(dir(0)/dir(3)-origin(0),
                                 dir(1)/dir(3)-origin(1),
                                 dir(2)/dir(3)-origin(2));
      std::cout << "ORG=(" << origin(0) << "," << origin(1) << "," <<
        origin(2) << ") DIR=(" << direction.x() << "," << direction.y() <<
        "," << direction.z() << ")" << std::endl;
      octomap::point3d hitPoint;
      if (mOctree->castRay(origin, direction, hitPoint, true, -1)) {
        Eigen::Vector4d pt(hitPoint.x(), hitPoint.y(), hitPoint.z(), 1);
        pt = xformToImage*pt;
        pt /= pt(3);
        depthMap.mData[index] = pt(2);  // TODO: make align with pix
        std::cout << "PT (" << j << "," << i << ") (" <<
          pt(0) << "," << pt(1) << "," << pt(2) << ")" << std::endl;
      }
      else {
        depthMap.mData[index] = unobservedValue;
      }
    }
  }
  
  return depthMap;
}

*/
