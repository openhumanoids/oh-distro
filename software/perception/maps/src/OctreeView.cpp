#include "OctreeView.hpp"

#include <octomap/octomap.h>
#include <pcl/common/transforms.h>

using namespace maps;

OctreeView::
OctreeView() {
  mOctree.reset(new octomap::OcTree(0.01));
}

OctreeView::
~OctreeView() {
}

void OctreeView::
setResolution(const float iResolution) {
  mOctree->setResolution(iResolution);
}

std::shared_ptr<octomap::OcTree> OctreeView::
getOctree() const {
  return mOctree;
}

const ViewBase::Type OctreeView::
getType() const {
  return TypeOctree;
}

ViewBase::Ptr OctreeView::
clone() const {
  OctreeView* view = new OctreeView(*this);
  view->mOctree.reset(new octomap::OcTree(*mOctree));
  return Ptr(view);
}

void OctreeView::
set(const maps::PointCloud::Ptr& iCloud) {
  Eigen::Affine3f xform(mTransform.matrix());
  for (int i = 0; i < iCloud->size(); ++i) {
    maps::PointCloud::PointType& inPt = (*iCloud)[i];
    /* NOTE: for projective
    Eigen::Vector4f projPt = mTransform*Eigen::Vector4f(inPt.x,inPt.y,inPt.z,1);
    Eigen::Vector3f pt(projPt[0], projPt[1], projPt[2]);
    pt /= projPt[3];
    */
    Eigen::Vector3f pt = xform*inPt.getVector3fMap();
    octomap::point3d octPt(pt[0], pt[1], pt[2]);
    mOctree->updateNode(octPt, true);
  }
}

maps::PointCloud::Ptr OctreeView::
getAsPointCloud(const bool iTransform) const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  octomap::OcTree::leaf_iterator iter = mOctree->begin_leafs();
  for (; iter != mOctree->end_leafs(); ++iter) {
    if (mOctree->isNodeOccupied(*iter)) {
      maps::PointCloud::PointType pt;
      pt.x = iter.getX();
      pt.y = iter.getY();
      pt.z = iter.getZ();
      /* NOTE: for projective
      Eigen::Vector4f projPt = mTransform*Eigen::Vector4f(pt.x,pt.y,pt.z,1);
      pt.getVector3fMap() = projPt.head<3>()/projPt[3];
      */
      cloud->push_back(pt);
    }
  }
  if (iTransform) {
    pcl::transformPointCloud(*cloud, *cloud,
                             Eigen::Affine3f(mTransform.matrix()));
  }
  return cloud;
}
