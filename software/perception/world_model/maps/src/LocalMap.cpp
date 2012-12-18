#include "LocalMap.hpp"

#include <octomap/octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

#include <limits>

using namespace maptypes;

LocalMap::
LocalMap() {
  setId(-1);
  setStateId(-1);
  setTransformToLocal(Eigen::Isometry3d::Identity());
  setBounds(Eigen::Vector3d(-1e10,-1e10,-1e10),
            Eigen::Vector3d(1e10,1e10,1e10));
  setResolution(0.1);
}

LocalMap::
~LocalMap() {
}

void LocalMap::
clear() {
  mOctree->clear();
}

void LocalMap::
setId(const int64_t iId) {
  mId = iId;
}

int64_t LocalMap::
getId() const {
  return mId;
}

void LocalMap::
setStateId(const int64_t iId) {
  mStateId = iId;
}

int64_t LocalMap::
getStateId() const {
  return mStateId;
}


void LocalMap::
setTransformToLocal(const Eigen::Isometry3d& iTransform) {
  mTransformToLocal = iTransform;
}

Eigen::Isometry3d LocalMap::
getTransformToLocal() const {
  return mTransformToLocal;
}

bool LocalMap::
setBounds(const Eigen::Vector3d& iMin, const Eigen::Vector3d& iMax) {
  if ((iMax[0] < iMin[0]) || (iMax[1] < iMin[1]) || (iMax[2] < iMin[2])) {
    return false;
  }
  mBoundMin = iMin;
  mBoundMax = iMax;
  return true;
}

Eigen::Vector3d LocalMap::
getBoundMin() const {
  return mBoundMin;
}

Eigen::Vector3d LocalMap::
getBoundMax() const {
  return mBoundMax;
}

void LocalMap::
setResolution(const double iResolution) {
  mResolution = iResolution;
  mOctree.reset(new Octree(mResolution));
  mOctree->enableChangeDetection(true);
}

double LocalMap::
getResolution() const {
  return mResolution;
}

bool LocalMap::
add(const PointCloud::Ptr& iPoints,
    const Eigen::Isometry3d& iToLocal,
    const bool iRayTraceFromOrigin) {

  // transform points
  PointCloud::Ptr points(new PointCloud());
  Eigen::Affine3f matx((mTransformToLocal.inverse()*iToLocal).cast<float>());
  pcl::transformPointCloud(*iPoints, *points, matx);

  // crop points
  pcl::CropBox<PointCloud::PointType> cropper;
  cropper.setMin(Eigen::Vector4f(mBoundMin[0], mBoundMin[1], mBoundMin[2], 1));
  cropper.setMax(Eigen::Vector4f(mBoundMax[0], mBoundMax[1], mBoundMax[2], 1));
  cropper.setInputCloud(points);
  PointCloud::Ptr pointsFinal(new PointCloud());
  cropper.filter(*pointsFinal);
  
  // use origin if desired
  if (iRayTraceFromOrigin) {
    octomap::Pointcloud octPoints;
    for (int i = 0; i < pointsFinal->points.size(); ++i) {
      octomap::point3d pt(pointsFinal->points[i].x,
                          pointsFinal->points[i].y,
                          pointsFinal->points[i].z);
      octPoints.push_back(pt);
    }
    // get and transform origin if desired
    Eigen::Vector3d origin =
      mTransformToLocal.inverse()*iToLocal.translation();
    octomap::point3d octOrigin(origin[0], origin[1], origin[2]);
    mOctree->insertScan(octPoints, octOrigin);
  }

  // otherwise just treat as individual points
  else {
    for (int i = 0; i < pointsFinal->points.size(); ++i) {
      octomap::point3d pt(pointsFinal->points[i].x,
                          pointsFinal->points[i].y,
                          pointsFinal->points[i].z);
      mOctree->updateNode(pt, true);
    }
  }

  return false;
}

PointCloud::Ptr LocalMap::
getAsPointCloud() const {
  // accumulate points
  PointCloud::Ptr cloud(new PointCloud());
  Octree::iterator iter;
  for (iter = mOctree->begin_leafs(); iter != mOctree->end_leafs(); ++iter) {
    bool occupied = mOctree->isNodeOccupied(*iter);
    if (!occupied) {
      continue;
    }
    PointCloud::PointType pt;
    pt.x = iter.getX();
    pt.y = iter.getY();
    pt.z = iter.getZ();
    cloud->points.push_back(pt);
  }

  // transform to local coords
  Eigen::Affine3f matx(mTransformToLocal.cast<float>());
  pcl::transformPointCloud(*cloud, *cloud, matx);
  
  return cloud;
}

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
  Eigen::Vector4d o = xformFromImage.matrix()*Eigen::Vector4d(0,0,1,0);
  octomap::point3d origin(o(0)/o(3), o(1)/o(3), o(2)/o(3));

  depthMap.mData.resize(iWidth*iHeight);
  const float unobservedValue = -std::numeric_limits<float>::max();
  for (int i = 0; i < iHeight; ++i) {
    for (int j = 0; j < iWidth; ++j) {
      int index = i*iWidth + j;
      Eigen::Vector4d dir = xformFromImage*Eigen::Vector4d(j,i,1,1);
      octomap::point3d direction(dir(0)/dir(3)-origin(0),
                                 dir(1)/dir(3)-origin(1),
                                 dir(2)/dir(3)-origin(2));
      octomap::point3d hitPoint;
      if (mOctree->castRay(origin, direction, hitPoint, false, -1)) {
        depthMap.mData[index] = (hitPoint-origin).norm()/direction.norm();
      }
      else {
        depthMap.mData[index] = unobservedValue;
      }
    }
  }
  
  return depthMap;
}

void LocalMap::
getAsRaw(std::vector<uint8_t>& oBytes) const {
  std::ostringstream oss;
  mOctree->writeBinaryConst(oss);
  std::string str = oss.str();
  oBytes.resize(str.size());
  std::copy(str.begin(), str.end(), oBytes.begin());
}

void LocalMap::
resetChangeReference() {
  mOctree->resetChangeDetection();
}

void LocalMap::
getChanges(PointCloud::Ptr& oAdded, PointCloud::Ptr& oRemoved) {
  if (oAdded == NULL) {
    oAdded.reset(new PointCloud());
  }
  if (oRemoved == NULL) {
    oRemoved.reset(new PointCloud());
  }
  oAdded->points.clear();
  oRemoved->points.clear();
  octomap::KeyBoolMap::const_iterator iter = mOctree->changedKeysBegin();
  for (; iter != mOctree->changedKeysEnd(); ++iter) {
    octomap::point3d pt = mOctree->keyToCoord(iter->first);
    octomap::OcTreeNode* node = mOctree->search(iter->first);
    bool occupied = mOctree->isNodeOccupied(node);
    PointType point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    // TODO: consider using another node representation rather than 3d points
    if (occupied) {
      oAdded->points.push_back(point);
    }
    else {
      oRemoved->points.push_back(point);
    }
  }
}

void LocalMap::
applyChanges(const PointCloud::Ptr& iAdded, const PointCloud::Ptr& iRemoved) {
  for (int i = 0; i < iAdded->points.size(); ++i) {
    octomap::point3d pt(iAdded->points[i].x,
                        iAdded->points[i].y,
                        iAdded->points[i].z);
    mOctree->updateNode(pt, true);
  }
  for (int i = 0; i < iRemoved->points.size(); ++i) {
    octomap::point3d pt(iRemoved->points[i].x,
                        iRemoved->points[i].y,
                        iRemoved->points[i].z);
    mOctree->updateNode(pt, false);
  }
}


void LocalMap::
serialize(std::vector<char>& oBytes) const {
  std::ostringstream oss(std::ios::binary);
  oss.write((char*)&mId, sizeof(mId));
  oss.write((char*)&mStateId, sizeof(mStateId));
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double val = mTransformToLocal(i,j);
      oss.write((char*)&val, sizeof(val));
    }
  }
  for (int i = 0; i < 3; ++i) {
    double val = mBoundMin[i];
    oss.write((char*)&val, sizeof(val));
  }
  for (int i = 0; i < 3; ++i) {
    double val = mBoundMax[i];
    oss.write((char*)&val, sizeof(val));
  }
  oss.write((char*)&mResolution, sizeof(mResolution));

  // write octree structure
  mOctree->writeBinaryConst(oss);

  // convert to vector of bytes
  std::string str = oss.str();
  oBytes = std::vector<char>(str.begin(), str.end());
}

void LocalMap::
deserialize(const std::vector<char>& iBytes) {
  // compute number of header bytes
  const int kNumStructureBytes =
    sizeof(mId) +          // id
    sizeof(mStateId) +     // state id
    16*sizeof(double) +    // transform
    3*sizeof(double) +     // bound min
    3*sizeof(double) +     // bound max
    sizeof(mResolution);   // resolution
  std::string str(iBytes.begin(), iBytes.end());

  // create stringstream
  std::istringstream iss(str, std::ios::binary);

  // read id
  int64_t id;
  iss.read((char*)&id, sizeof(id));
  setId(id);

  // read state id
  iss.read((char*)&id, sizeof(id));
  setStateId(id);

  // read transform
  Eigen::Isometry3d transform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double val;
      iss.read((char*)&val, sizeof(val));
      transform(i,j) = val;
    }
  }
  setTransformToLocal(transform);

  // read bounds
  Eigen::Vector3d boundMin, boundMax;
  for (int i = 0; i < 3; ++i) {
    double val;
    iss.read((char*)&val, sizeof(val));
    boundMin[i] = val;
  }
  for (int i = 0; i < 3; ++i) {
    double val;
    iss.read((char*)&val, sizeof(val));
    boundMax[i] = val;
  }
  setBounds(boundMin, boundMax);

  // read resolution
  double resolution;
  iss.read((char*)&resolution, sizeof(resolution));
  setResolution(resolution);

  // read remaining octree structure
  mOctree->readBinary(iss);
}
