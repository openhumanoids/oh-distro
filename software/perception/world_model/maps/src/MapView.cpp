#include "MapView.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <octomap/octomap.h>

using namespace maps;

MapView::Spec::
Spec() {
  mMapId = mViewId = 0;
  mActive = false;
  mRelativeTime = false;
  mRelativeLocation = false;
  mType = TypePointCloud;
  mResolution = 0;
  mFrequency = 0;
  mTimeMin = mTimeMax = 0;
}

bool MapView::Spec::
operator==(const Spec& iSpec) const {
  bool eq = (mMapId == iSpec.mMapId) &&
    (mViewId == iSpec.mViewId) &&
    (mActive == iSpec.mActive) &&
    (mRelativeTime == iSpec.mRelativeTime) &&
    (mRelativeLocation == iSpec.mRelativeLocation) &&
    (mType == iSpec.mType) &&
    (mResolution == iSpec.mResolution) &&
    (mFrequency == iSpec.mFrequency) &&
    (mTimeMin == iSpec.mTimeMin) &&
    (mTimeMax == iSpec.mTimeMax) &&
    (mClipPlanes.size() == iSpec.mClipPlanes.size());
  if (!eq) {
    return false;
  }
  for (int i = 0; i < mClipPlanes.size(); ++i) {
    if (mClipPlanes[i] != iSpec.mClipPlanes[i]) {
      return false;
    }
  }
  return true;
}

bool MapView::Spec::
operator!=(const Spec& iSpec) const {
  return !(*this == iSpec);
}




MapView::
MapView(const Spec& iSpec) {
  mSpec = iSpec;
  mUpdateTime = 0;
  mCloud.reset(new maps::PointCloud());
}

MapView::
~MapView() {
}

MapView::Spec MapView::
getSpec() const {
  return mSpec;
}

bool MapView::
set(const maps::PointCloud& iCloud) {
  boost::mutex::scoped_lock lock(mMutex);
  mCloud.reset(new maps::PointCloud(iCloud));
  return true;
}

bool MapView::
set(const maps::Octree& iTree) {
  boost::mutex::scoped_lock lock(mMutex);
  mCloud.reset(new maps::PointCloud());
  octomap::OcTree::leaf_iterator iter = iTree.mTree->begin_leafs();
  for (; iter != iTree.mTree->end_leafs(); ++iter) {
    if (iTree.mTree->isNodeOccupied(*iter)) {
      maps::PointCloud::PointType pt;
      pt.x = iter.getX();
      pt.y = iter.getY();
      pt.z = iter.getZ();
      mCloud->push_back(pt);
    }
  }
  pcl::transformPointCloud(*mCloud, *mCloud, iTree.mTransform);
  return true;
}

void MapView::
setUpdateTime(const int64_t iTime) {
  mUpdateTime = iTime;
}

int64_t MapView::
getUpdateTime() const {
  return mUpdateTime;
}

MapView::Ptr MapView::
clone() const {
  return clone(mSpec);
}

MapView::Ptr MapView::
clone(const Spec& iSpec) const {
  boost::mutex::scoped_lock lock(mMutex);
  Ptr view(new MapView(iSpec));
  view->mCloud = mCloud;
  view->mUpdateTime = mUpdateTime;
  // TODO view->mFilters = mFilters;
  return view;
}

/*
bool MapView::
applyFilters(const maps::PointCloud::Ptr& iCloud,
             maps::PointCloud::Ptr& oCloud) const {
  bool success = true;
  if (oCloud == NULL) {
    oCloud.reset(new maps::PointCloud());
  }
  maps::PointCloud::Ptr tempCloud(new maps::PointCloud());
  maps::PointCloud::Ptr inCloud = iCloud;
  for (std::list<Filter::Ptr>::const_iterator iter = mFilters.begin();
       iter != mFilters.end(); ++iter) {
    if (!(*iter)->operate(*inCloud, *tempCloud)) {
      success = false;
      break;
    }
    inCloud = tempCloud;
    std::swap(tempCloud, oCloud);
  }
  oCloud = inCloud;
  return success;
}
*/

maps::PointCloud::Ptr MapView::
getAsPointCloud(const Filter::Ptr& iFilter) const {
  maps::PointCloud::Ptr cloud;
  {
    boost::mutex::scoped_lock lock(mMutex);
    cloud.reset(new maps::PointCloud(*mCloud));
  }
  // TODO: make safe for in-place operators
  if (iFilter != NULL) {
    iFilter->operate(*cloud, *cloud);
  }
  return cloud;
}

MapView::HeightMap::Ptr MapView::
getAsHeightMap(const float iResolution, const float iMaxHeight,
               const Filter::Ptr& iFilter) const {
  // get points
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  {
    boost::mutex::scoped_lock lock(mMutex);
    *cloud = *mCloud;
  }
  if (iFilter != NULL) {
    iFilter->operate(*cloud, *cloud);
  }

  // determine 2d extents
  maps::PointCloud::PointType minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);

  // compute transform from image to reference coordinates
  Eigen::Affine3f imgToRef = Eigen::Affine3f::Identity();
  imgToRef(0,0) = imgToRef(1,1) = iResolution;
  imgToRef(0,3) = minPoint.x;
  imgToRef(1,3) = minPoint.y;
  Eigen::Affine3f refToImg = imgToRef.inverse();
  
  // initialize height map data
  HeightMap::Ptr heightMap(new HeightMap());
  const float unobservedValue = -std::numeric_limits<float>::max();
  heightMap->mTransform = imgToRef;
  heightMap->mWidth = ceil((maxPoint.x - minPoint.x)/iResolution) + 1;
  heightMap->mHeight = ceil((maxPoint.y - minPoint.y)/iResolution) + 1;
  int totalPixels = heightMap->mWidth * heightMap->mHeight;
  heightMap->mData.resize(totalPixels);
  std::fill(heightMap->mData.begin(), heightMap->mData.end(), unobservedValue);

  // add height values into height map
  // TODO: consider case of going from lower res to higher res
  pcl::transformPointCloud(*cloud, *cloud, refToImg);
  for (int i = 0; i < cloud->size(); ++i) {
    int x((*cloud)[i].x + 0.5f), y((*cloud)[i].y + 0.5f);
    int index = y*heightMap->mWidth + x;
    heightMap->mData[index] = std::max(heightMap->mData[index], (*cloud)[i].z);
  }

  return heightMap;
}

MapView::TriangleMesh::Ptr MapView::
getAsMesh(const Filter::Ptr& iFilter) const {
  // get points
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  {
    boost::mutex::scoped_lock lock(mMutex);
    *cloud = *mCloud;
  }
  if (iFilter != NULL) {
    iFilter->operate(*cloud, *cloud);
  }
  typedef pcl::PointXYZ PointType;
  pcl::PointCloud<PointType>::Ptr cloudXyz(new pcl::PointCloud<PointType>());
  pcl::copyPointCloud(*cloud, *cloudXyz);

  // Normal estimation
  pcl::NormalEstimation<PointType, pcl::Normal> normEst;
  pcl::PointCloud<pcl::Normal>::Ptr
    normals(new pcl::PointCloud<pcl::Normal>());
  pcl::search::KdTree<PointType>::Ptr
    tree(new pcl::search::KdTree<PointType>());
  tree->setInputCloud(cloudXyz);
  normEst.setInputCloud(cloudXyz);
  normEst.setSearchMethod(tree);
  normEst.setKSearch(10);
  normEst.compute(*normals);

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr
    cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::concatenateFields(*cloudXyz, *normals, *cloudWithNormals);

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr
    tree2(new pcl::search::KdTree<pcl::PointNormal>());
  tree2->setInputCloud(cloudWithNormals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set values for the parameters
  const double kPi = 4*atan(1);
  const double degToRad = kPi/180;
  const float resolution = (mSpec.mResolution>0 ? mSpec.mResolution : 0.1);
  gp3.setSearchRadius(20*resolution);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(50);
  gp3.setMaximumSurfaceAngle(45*degToRad);
  gp3.setMinimumAngle(10*degToRad);
  gp3.setMaximumAngle(120*degToRad);
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(cloudWithNormals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  // convert result
  TriangleMesh::Ptr mesh(new TriangleMesh);
  maps::PointCloud cloudTemp;
  pcl::fromROSMsg(triangles.cloud, cloudTemp);
  mesh->mVertices.resize(cloudTemp.size());
  for (int i = 0; i < cloudTemp.size(); ++i) {
    mesh->mVertices[i] = Eigen::Vector3f(cloudTemp[i].x, cloudTemp[i].y,
                                         cloudTemp[i].z);
  }
  mesh->mFaces.resize(triangles.polygons.size());
  for (int i = 0; i < triangles.polygons.size(); ++i) {
    std::vector<uint32_t>& vtx = triangles.polygons[i].vertices;
    mesh->mFaces[i] = Eigen::Vector3i(vtx[0], vtx[1], vtx[2]);
  }
  return mesh;
}

MapView::SurfelCloud::Ptr
MapView::getAsSurfels(const Filter::Ptr& iFilter) const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  {
    boost::mutex::scoped_lock lock(mMutex);
    *cloud = *mCloud;
  }
  if (iFilter != NULL) {
    iFilter->operate(*cloud, *cloud);
  }

  // Normal estimation
  pcl::NormalEstimation<maps::PointType, pcl::Normal> normEst;
  pcl::PointCloud<pcl::Normal>::Ptr
    normals(new pcl::PointCloud<pcl::Normal>());
  pcl::search::KdTree<maps::PointType>::Ptr
    tree(new pcl::search::KdTree<maps::PointType>());
  tree->setInputCloud(cloud);
  normEst.setInputCloud(cloud);
  normEst.setSearchMethod(tree);
  normEst.setKSearch(10);
  normEst.compute(*normals);

  SurfelCloud::Ptr surfels(new SurfelCloud());
  surfels->resize(cloud->size());
  for (int i = 0; i < surfels->size(); ++i) {
    pcl::PointSurfel& surf = (*surfels)[i];
    maps::PointType& pt = (*cloud)[i];
    pcl::Normal& normal = (*normals)[i];
    surf.x = pt.x;
    surf.y = pt.y;
    surf.z = pt.z;
    for (int k = 0; k < 3; ++k) surf.normal[k] = normal.normal[k];
    surf.curvature = normal.curvature;
    surf.radius = 1;  // TODO: can we estimate this easily?
  }
  return surfels;
}
