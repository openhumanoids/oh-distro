#include "MapChunk.hpp"

#include <pcl/octree/octree_impl.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

MapChunk::
MapChunk() {
  setId(-1);
  setTransformToLocal(Eigen::Isometry3d::Identity());
  setBounds(Eigen::Vector3d(-1e20, -1e20, -1e20),
            Eigen::Vector3d(1e20, 1e20, 1e20));
  setResolution(0.01);
}

MapChunk::
~MapChunk() {
}

void MapChunk::
clear() {
  mOctree->deleteTree();
}

void MapChunk::
setId(const int64_t iId) {
  mId = iId;
}

int64_t MapChunk::
getId() const {
  return mId;
}

void MapChunk::
setTransformToLocal(const Eigen::Isometry3d& iTransform) {
  mTransformToLocal = iTransform;
}

Eigen::Isometry3d MapChunk::
getTransformToLocal() const {
  return mTransformToLocal;
}

bool MapChunk::
setBounds(const Eigen::Vector3d& iMin, const Eigen::Vector3d& iMax) {
  if ((iMin[0] > iMax[0]) || (iMin[1] > iMax[1]) || (iMin[2] > iMax[2])) {
    return false;
  }
  mBoundMin = iMin;
  mBoundMax = iMax;
  return true;
}

Eigen::Vector3d MapChunk::
getBoundMin() const {
  return mBoundMin;
}

Eigen::Vector3d MapChunk::
getBoundMax() const {
  return mBoundMax;
}

void MapChunk::
setResolution(const double iResolution) {
  mResolution = iResolution;
  mOctree.reset(new Octree(mResolution));
}

double MapChunk::
getResolution() const {
  return mResolution;
}

bool MapChunk::
add(const PointCloud::Ptr& iPoints, const Eigen::Isometry3d& iToLocal) {

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
  mOctree->setOccupiedVoxelsAtPointsFromCloud(pointsFinal);

  return true;
}

bool MapChunk::
remove(const PointCloud::Ptr& iCloud) {
  PointCloud::Ptr newCloud(new PointCloud());
  Eigen::Affine3f matx(mTransformToLocal.inverse().cast<float>());
  pcl::transformPointCloud(*iCloud, *newCloud, matx);
  for (int i = 0; i < newCloud->size(); ++i) {
    mOctree->deleteVoxelAtPoint(newCloud->points[i]);
  }
  return true;
}

void MapChunk::
deepCopy(const MapChunk& iChunk) {
  OctreePtr octreeOrig = mOctree;
  *this = iChunk;
  mOctree = octreeOrig;
  *mOctree = *(iChunk.mOctree);
}

MapChunk::PointCloud::Ptr MapChunk::
getAsPointCloud(const bool iTransform) const {
  PointCloud::Ptr cloud(new PointCloud());
  mOctree->getOccupiedVoxelCenters(cloud->points);
  if (iTransform) {
    Eigen::Affine3f matx(mTransformToLocal.cast<float>());
    pcl::transformPointCloud(*cloud, *cloud, matx);
  }
  return cloud;
}

bool MapChunk::
findDifferences(const MapChunk& iMap, PointCloud::Ptr& oAdded,
                PointCloud::Ptr& oRemoved) {

  // find newly added points
  if (oAdded == NULL) {
    oAdded.reset(new PointCloud());
  }
  oAdded->clear();
  double boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ;
  iMap.mOctree->getBoundingBox(boundMinX, boundMinY, boundMinZ,
                               boundMaxX, boundMaxY, boundMaxZ);
  double resolution = iMap.mOctree->getResolution();
  Eigen::Vector3d boundMin(boundMinX, boundMinY, boundMinZ);
  {
    Octree::LeafNodeIterator iter(*iMap.mOctree);
    while (*++iter) {
      pcl::octree::OctreeKey key = iter.getCurrentOctreeKey();
      if (mOctree->existLeaf(key.x, key.y, key.z)) {
        continue;
      }

      Eigen::Vector3d pt(key.x, key.y, key.z);
      pt = (pt.array() + 0.5) * resolution;
      pt += boundMin;
      oAdded->push_back(PointCloud::PointType(pt[0], pt[1], pt[2]));
    }
  }

  // find newly removed points
  if (oRemoved == NULL) {
    oRemoved.reset(new PointCloud());
  }
  oRemoved->clear();
  mOctree->getBoundingBox(boundMinX, boundMinY, boundMinZ,
                          boundMaxX, boundMaxY, boundMaxZ);
  resolution = mOctree->getResolution();
  boundMin = Eigen::Vector3d(boundMinX, boundMinY, boundMinZ);
  {
    Octree::LeafNodeIterator iter(*iMap.mOctree);
    while (*++iter) {
      pcl::octree::OctreeKey key = iter.getCurrentOctreeKey();
      if (iMap.mOctree->existLeaf(key.x, key.y, key.z)) {
        continue;
      }

      Eigen::Vector3d pt(key.x, key.y, key.z);
      pt = (pt.array() + 0.5) * resolution;
      pt += boundMin;
      oRemoved->push_back(PointCloud::PointType(pt[0], pt[1], pt[2]));
    }
  }

  return true;
}

void MapChunk::
serialize(std::vector<char>& oBytes) const {
  // write member bytes to string stream
  std::ostringstream oss(std::ios::binary);
  oss.write((char*)&mId, sizeof(mId));
  oss.write((char*)&mResolution, sizeof(mResolution));
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

  // serialize octree attributes
  int treeDepth = mOctree->getTreeDepth();
  oss.write((char*)&treeDepth, sizeof(treeDepth));
  double xMin, yMin, zMin, xMax, yMax, zMax;
  mOctree->getBoundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
  oss.write((char*)&xMin, sizeof(xMin));
  oss.write((char*)&yMin, sizeof(yMin));
  oss.write((char*)&zMin, sizeof(zMin));
  oss.write((char*)&xMax, sizeof(xMax));
  oss.write((char*)&yMax, sizeof(yMax));
  oss.write((char*)&zMax, sizeof(zMax));

  // convert to vector
  std::string str = oss.str();
  oBytes = std::vector<char>(str.begin(), str.end());

  // add octree
  std::vector<char> treeBytes;
  mOctree->serializeTree(treeBytes);
  oBytes.insert(oBytes.end(), treeBytes.begin(), treeBytes.end());
  
  // TODO: could use lcm marshalling or boost serialization for this
}

void MapChunk::
deserialize(const std::vector<char>& iBytes) {
  const int kNumStructureBytes = sizeof(mId) + sizeof(mResolution) +
    16*sizeof(double) + 3*sizeof(double) + 3*sizeof(double) +
    sizeof(int) + 6*sizeof(double);
  std::string str(iBytes.begin(), iBytes.begin()+kNumStructureBytes);
  std::istringstream iss(str, std::ios::binary);

  iss.read((char*)&mId, sizeof(mId));
  iss.read((char*)&mResolution, sizeof(mResolution));
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double val;
      iss.read((char*)&val, sizeof(val));
      mTransformToLocal(i,j) = val;
    }
  }
  for (int i = 0; i < 3; ++i) {
    double val;
    iss.read((char*)&val, sizeof(val));
    mBoundMin[i] = val;
  }
  for (int i = 0; i < 3; ++i) {
    double val;
    iss.read((char*)&val, sizeof(val));
    mBoundMax[i] = val;
  }

  // read octree attributes
  int treeDepth;
  iss.read((char*)&treeDepth, sizeof(treeDepth));
  double xMin, yMin, zMin, xMax, yMax, zMax;
  iss.read((char*)&xMin, sizeof(xMin));
  iss.read((char*)&yMin, sizeof(yMin));
  iss.read((char*)&zMin, sizeof(zMin));
  iss.read((char*)&xMax, sizeof(xMax));
  iss.read((char*)&yMax, sizeof(yMax));
  iss.read((char*)&zMax, sizeof(zMax));

  // construct octree
  std::vector<char> treeBytes(iBytes.begin()+kNumStructureBytes,
                              iBytes.end());
  setResolution(mResolution);
  mOctree->setTreeDepth(treeDepth);
  mOctree->defineBoundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
  mOctree->deserializeTree(treeBytes);
}
