#include "LidarFilters.hpp"

#include <maps/Utils.hpp>

using namespace maps;

TorsoFilter::
TorsoFilter(const BotWrapper::Ptr& iBotWrapper) {
  mBotWrapper = iBotWrapper;
}

void TorsoFilter::
operator()(maps::PointSet& ioPoints) {
  // compute scan to torso transform
  Eigen::Isometry3f torsoToLocal;
  mBotWrapper->getTransform("utorso","local",torsoToLocal);
  Eigen::Isometry3f scanToLocal = Utils::getPose(*ioPoints.mCloud);
  Eigen::Isometry3f scanToTorso = torsoToLocal.inverse()*scanToLocal;

  // clip points against torso
  int numPoints = ioPoints.mCloud->size();
  std::vector<int> goodIndices;
  goodIndices.reserve(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    const maps::PointType& pt = (*ioPoints.mCloud)[i];
    Eigen::Vector3f p = scanToTorso*Eigen::Vector3f(pt.x,pt.y,pt.z);
    // NOTE: these bounds were taken from urdf 2013-12-09
    bool bad = ((p[0] < 0.3) && (p[0] > -0.2) && (fabs(p[1]) < 0.25) &&
                (p[2] < 0.75) && (p[2] > 0));
    if (!bad) goodIndices.push_back(i);
  }

  // create new points
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  cloud->resize(goodIndices.size());
  cloud->width = goodIndices.size();
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->sensor_origin_ = ioPoints.mCloud->sensor_origin_;
  cloud->sensor_orientation_ = ioPoints.mCloud->sensor_orientation_;
  for (int i = 0; i < goodIndices.size(); ++i) {
    (*cloud)[i] = (*ioPoints.mCloud)[goodIndices[i]];
  }
  ioPoints.mCloud = cloud;
}

GroundFilter::
GroundFilter(const BotWrapper::Ptr& iBotWrapper) {
  mBotWrapper = iBotWrapper;
  mActive = false;
  mBotWrapper->getLcm()->subscribe
    ("MAP_DEPTH_SETTINGS", &GroundFilter::onSettings, this);
}

void GroundFilter::
onSettings(const lcm::ReceiveBuffer* iBuf,
           const std::string& iChannel,
           const drc::map_depth_settings_t* iMessage) {
  mActive = iMessage->remove_ground;
}


void GroundFilter::
operator()(maps::PointSet& ioPoints) {
  if (!mActive) return;
  
  // transform ground plane to scan coords
  Eigen::Isometry3f groundToLocal;
  mBotWrapper->getTransform("ground","local",groundToLocal);
  Eigen::Isometry3f scanToLocal = Utils::getPose(*ioPoints.mCloud);
  Eigen::Isometry3f scanToGround = groundToLocal.inverse()*scanToLocal;
  Eigen::Matrix4f groundPlaneToScan = scanToGround.matrix().transpose();
  const float kDistanceThresh = 0.1;
  Eigen::Vector4f plane = groundPlaneToScan*Eigen::Vector4f(0,0,1,0);

  // include only points higher than threshold above ground
  int numPoints = ioPoints.mCloud->size();
  std::vector<int> goodIndices;
  goodIndices.reserve(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    const maps::PointType& pt = (*ioPoints.mCloud)[i];
    float dist = plane[0]*pt.x + plane[1]*pt.y + plane[2]*pt.z + plane[3];
    if (dist > kDistanceThresh) goodIndices.push_back(i);
  }

  // create new points
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  cloud->resize(goodIndices.size());
  cloud->width = goodIndices.size();
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->sensor_origin_ = ioPoints.mCloud->sensor_origin_;
  cloud->sensor_orientation_ = ioPoints.mCloud->sensor_orientation_;
  for (int i = 0; i < goodIndices.size(); ++i) {
    (*cloud)[i] = (*ioPoints.mCloud)[goodIndices[i]];
  }
  ioPoints.mCloud = cloud;
}
