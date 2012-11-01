#include "LocalMap.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

LocalMap::
LocalMap() {
  setId(-1);
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
    const Eigen::Isometry3d& iToLocal) {

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
  
  // add points
  for (int i = 0; i < pointsFinal->points.size(); ++i) {
    octomap::point3d pt(pointsFinal->points[i].x,
                        pointsFinal->points[i].y,
                        pointsFinal->points[i].z);
    mOctree->updateNode(pt, true);
  }

  return true;
}

bool LocalMap::
add(const PointCloud::Ptr& iPoints,
    const Eigen::Vector3d& iOrigin,
    const Eigen::Isometry3d& iToLocal) {

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
  
  // add points
  octomap::Pointcloud octPoints;
  for (int i = 0; i < pointsFinal->points.size(); ++i) {
    octomap::point3d pt(pointsFinal->points[i].x,
                        pointsFinal->points[i].y,
                        pointsFinal->points[i].z);
    octPoints.push_back(pt);
  }
  octomap::point3d origin(iOrigin[0], iOrigin[1], iOrigin[2]);
  mOctree->insertScan(octPoints, origin);

  return false;
}

LocalMap::PointCloud::Ptr LocalMap::
getAsPointCloud(const bool iTransform) const {
  PointCloud::Ptr cloud(new PointCloud());
  Octree::iterator iter;
  int numPoints = mOctree->calcNumNodes();
  cloud->points.reserve(numPoints);
  for (iter = mOctree->begin_leafs(); iter != mOctree->end_leafs(); ++iter) {
    bool occupied = mOctree->isNodeOccupied(*iter);
    if (!occupied) {
      continue;
    }
    PointCloud::PointType pt(iter.getX(), iter.getY(), iter.getZ());
    cloud->points.push_back(pt);
  }
  return cloud;
}

LocalMap::HeightMap LocalMap::
getAsHeightMap() const {
  const double unobservedValue = -1e10;
  HeightMap heightMap;

  // determine 2d extents of octree (x,y) in units of cells
  // TODO: could also create a hash map of occupied keys
  // TODO: ... and do everything in a single pass
  Octree::iterator iter;
  int xMin(65536), yMin(65536), xMax(-65536), yMax(-65536);
  for (iter = mOctree->begin_leafs(); iter != mOctree->end_leafs(); ++iter) {
    octomap::OcTreeKey key = iter.getKey();
    bool occupied = mOctree->isNodeOccupied(*iter);
    if (!occupied) {
      continue;
    }
    xMin = std::min(xMin, (int)key[0]);
    yMin = std::min(yMin, (int)key[1]);
    xMax = std::max(xMax, (int)key[0]);
    yMax = std::max(yMax, (int)key[1]);
  }

  // determine transform from image to local coordinates
  double scale = mOctree->getResolution();
  double offset = mOctree->keyToCoord(0);
  Eigen::Affine2d xform = Eigen::Affine2d::Identity();
  xform(0,0) = xform(1,1) = scale;
  xform(0,2) = offset + xMin*scale;
  xform(1,2) = offset + yMin*scale;
  heightMap.mTransformToLocal = xform;
  
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
    double z = iter.getZ();
    int index = (key[1]-yMin)*heightMap.mWidth + (key[0]-xMin);
    bool occupied = mOctree->isNodeOccupied(*iter);
    if (!occupied) {
      continue;
    }
    heightMap.mData[index] = std::max(heightMap.mData[index], z);
  }

  return heightMap;
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
    if (occupied) {
      oAdded->points.push_back(PointCloud::PointType(pt.x(), pt.y(), pt.z()));
    }
    else {
      oRemoved->points.push_back(PointCloud::PointType(pt.x(), pt.y(), pt.z()));
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
  mOctree->write(oss);

  // convert to vector of bytes
  std::string str = oss.str();
  oBytes = std::vector<char>(str.begin(), str.end());
}

void LocalMap::
deserialize(const std::vector<char>& iBytes) {
  // compute number of header bytes
  const int kNumStructureBytes =
    sizeof(mId) +          // id
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
  octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(iss);
  mOctree.reset((Octree*)tree);
}
