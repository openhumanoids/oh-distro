#include "BlockFitter.hpp"

#include <chrono>
#include <fstream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>

#include "RobustNormalEstimator.hpp"
#include "PlaneSegmenter.hpp"
#include "RectangleFitter.hpp"

using namespace planeseg;

BlockFitter::
BlockFitter() {
  setSensorPose(Eigen::Vector3f(0,0,0), Eigen::Vector3f(1,0,0));
  setBlockDimensions(Eigen::Vector3f(15+3/8.0, 15+3/8.0, 5+5/8.0)*0.0254);
  setDownsampleResolution(0.01);
  setRemoveGround(true);
  setHeightBand(0.05, 1.0);
  setMaxRange(3.0);
  setMaxAngleFromHorizontal(45);
  setDebug(true);
}

void BlockFitter::
setSensorPose(const Eigen::Vector3f& iOrigin,
              const Eigen::Vector3f& iLookDir) {
  mOrigin = iOrigin;
  mLookDir = iLookDir;
}

void BlockFitter::
setBlockDimensions(const Eigen::Vector3f& iDimensions) {
  mBlockDimensions = iDimensions;
}

void BlockFitter::
setDownsampleResolution(const float iRes) {
  mDownsampleResolution = iRes;
}

void BlockFitter::
setRemoveGround(const bool iVal) {
  mRemoveGround = iVal;
}

void BlockFitter::
setHeightBand(const float iMinHeight, const float iMaxHeight) {
  mMinHeightAboveGround = iMinHeight;
  mMaxHeightAboveGround = iMaxHeight;
}

void BlockFitter::
setMaxRange(const float iRange) {
  mMaxRange = iRange;
}

void BlockFitter::
setMaxAngleFromHorizontal(const float iDegrees) {
  mMaxAngleFromHorizontal = iDegrees;
}

void BlockFitter::
setCloud(const LabeledCloud::Ptr& iCloud) {
  mCloud = iCloud;
}

void BlockFitter::
setDebug(const bool iVal) {
  mDebug = iVal;
}

BlockFitter::Result BlockFitter::
go() {
  Result result;

  // voxelize
  LabeledCloud::Ptr cloud(new LabeledCloud());
  pcl::VoxelGrid<pcl::PointXYZL> voxelGrid;
  voxelGrid.setInputCloud(mCloud);
  voxelGrid.setLeafSize(mDownsampleResolution, mDownsampleResolution,
                        mDownsampleResolution);
  voxelGrid.filter(*cloud);
  for (int i = 0; i < (int)cloud->size(); ++i) cloud->points[i].label = i;

  if (mDebug) {
    std::cout << "Original cloud size " << mCloud->size() << std::endl;
    std::cout << "Voxelized cloud size " << cloud->size() << std::endl;
    pcl::io::savePCDFileBinary("cloud_full.pcd", *cloud);
  }

  // pose
  cloud->sensor_origin_.head<3>() = mOrigin;
  cloud->sensor_origin_[3] = 1;
  Eigen::Vector3f rz = mLookDir;
  Eigen::Vector3f rx = rz.cross(Eigen::Vector3f::UnitZ());
  Eigen::Vector3f ry = rz.cross(rx);
  Eigen::Matrix3f rotation;
  rotation.col(0) = rx.normalized();
  rotation.col(1) = ry.normalized();
  rotation.col(2) = rz.normalized();
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.linear() = rotation;
  pose.translation() = mOrigin;

  // ground removal
  if (mRemoveGround) {
    Eigen::Vector4f groundPlane;

    // filter points
    std::vector<float> zVals(cloud->size());
    for (int i = 0; i < (int)cloud->size(); ++i) zVals[i] = cloud->points[i].z;
    std::sort(zVals.begin(), zVals.end());
    //float minZ = zVals[(int)(zVals.size()*0.01)]; 
    //float heightThresh = minZ + mMaxHeightAboveGround;
    float minZ = zVals[0];
    float heightThresh = minZ + mMaxHeightAboveGround/2;
    std::cout << minZ << " " << zVals[0] << " " << heightThresh << std::endl;
    LabeledCloud::Ptr tempCloud(new LabeledCloud());
    for (int i = 0; i < (int)cloud->size(); ++i) {
      const Eigen::Vector3f& p = cloud->points[i].getVector3fMap();
      if (p[2] > heightThresh) continue;
      tempCloud->push_back(cloud->points[i]);
    }

    // find ground plane
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZL> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(tempCloud);
    seg.segment(*inliers, *coeffs);
    groundPlane << coeffs->values[0], coeffs->values[1],
      coeffs->values[2], coeffs->values[3];
    if (groundPlane[2] < 0) groundPlane = -groundPlane;
    if (mDebug) {
      std::cout << "dominant plane: " << groundPlane.transpose() << std::endl;
      std::cout << "  inliers: " << inliers->indices.size() << std::endl;
    }
    if (std::acos(groundPlane[2]) > 30*M_PI/180) {
      std::cout << "error: ground plane not found!" << std::endl;
      std::cout << "proceeding with full segmentation (may take a while)..." <<
        std::endl;
    }
    else {
      tempCloud.reset(new LabeledCloud());
      for (int i = 0; i < (int)cloud->size(); ++i) {
        Eigen::Vector3f p = cloud->points[i].getVector3fMap();
        float dist = p.dot(groundPlane.head<3>()) + groundPlane[3];
        if ((dist < mMinHeightAboveGround) ||
            (dist > mMaxHeightAboveGround)) continue;
        float range = (p-mOrigin).norm();
        if (range > mMaxRange) continue;
        tempCloud->push_back(cloud->points[i]);
      }
      std::swap(tempCloud, cloud);
      if (mDebug) {
        std::cout << "Filtered cloud size " << cloud->size() << std::endl;
      }
    }
  }

  // normal estimation
  auto t0 = std::chrono::high_resolution_clock::now();
  if (mDebug) {
    std::cout << "computing normals..." << std::flush;
  }
  RobustNormalEstimator normalEstimator;
  normalEstimator.setMaxEstimationError(0.01);
  normalEstimator.setRadius(0.1);
  normalEstimator.setMaxCenterError(0.02);
  normalEstimator.setMaxIterations(100);
  NormalCloud::Ptr normals(new NormalCloud());
  normalEstimator.go(cloud, *normals);
  if (mDebug) {
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
    std::cout << "finished in " << dt.count()/1e3 << " sec" << std::endl;
  }

  // filter non-horizontal points
  const float maxNormalAngle = mMaxAngleFromHorizontal*M_PI/180;
  LabeledCloud::Ptr tempCloud(new LabeledCloud());
  NormalCloud::Ptr tempNormals(new NormalCloud());
  for (int i = 0; i < (int)normals->size(); ++i) {
    const auto& norm = normals->points[i];
    Eigen::Vector3f normal(norm.normal_x, norm.normal_y, norm.normal_z);
    float angle = std::acos(normal[2]);
    if (angle > maxNormalAngle) continue;
    tempCloud->push_back(cloud->points[i]);
    tempNormals->push_back(normals->points[i]);
  }
  std::swap(tempCloud, cloud);
  std::swap(tempNormals, normals);

  if (mDebug) {
    std::cout << "Horizontal points remaining " << cloud->size() << std::endl;
    pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
    pcl::io::savePCDFileBinary("robust_normals.pcd", *normals);
  }

  // plane segmentation
  t0 = std::chrono::high_resolution_clock::now();
  if (mDebug) {
    std::cout << "segmenting planes..." << std::flush;
  }
  PlaneSegmenter segmenter;
  segmenter.setData(cloud, normals);
  segmenter.setMaxError(0.05);
  segmenter.setMaxAngle(5);
  segmenter.setMinPoints(100);
  PlaneSegmenter::Result segmenterResult = segmenter.go();
  if (mDebug) {
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
    std::cout << "finished in " << dt.count()/1e3 << " sec" << std::endl;

    std::ofstream ofs("labels.txt");
    for (const int lab : segmenterResult.mLabels) {
      ofs << lab << std::endl;
    }
    ofs.close();

    ofs.open("planes.txt");
    for (auto it : segmenterResult.mPlanes) {
      auto& plane = it.second;
      ofs << it.first << " " << plane.transpose() << std::endl;
    }
    ofs.close();
  }

  // create point clouds
  std::unordered_map<int,std::vector<Eigen::Vector3f>> cloudMap;
  for (int i = 0; i < (int)segmenterResult.mLabels.size(); ++i) {
    int label = segmenterResult.mLabels[i];
    if (label <= 0) continue;
    cloudMap[label].push_back(cloud->points[i].getVector3fMap());
  }
  struct Plane {
    MatrixX3f mPoints;
    Eigen::Vector4f mPlane;
  };
  std::vector<Plane> planes;
  planes.reserve(cloudMap.size());
  for (auto it : cloudMap) {
    int n = it.second.size();
    Plane plane;
    plane.mPoints.resize(n,3);
    for (int i = 0; i < n; ++i) plane.mPoints.row(i) = it.second[i];
    plane.mPlane = segmenterResult.mPlanes[it.first];
    planes.push_back(plane);
  }

  std::vector<RectangleFitter::Result> results;
  for (auto& plane : planes) {
    RectangleFitter fitter;
    fitter.setDimensions(mBlockDimensions.head<2>());
    fitter.setData(plane.mPoints, plane.mPlane);
    auto result = fitter.go();
    results.push_back(result);
  }

  if (mDebug) {
    std::ofstream ofs("boxes.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      for (auto& p : res.mPolygon) {
        ofs << i << " " << p.transpose() << std::endl;
      }
    }
    ofs.close();

    ofs.open("hulls.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      for (auto& p : res.mConvexHull) {
        ofs << i << " " << p.transpose() << std::endl;
      }
    }
    ofs.close();

    ofs.open("poses.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      auto transform = res.mPose;
      ofs << transform.matrix() << std::endl;
    }
    ofs.close();
  }

  result.mBlocks.resize(results.size());
  for (int i = 0; i < (int)results.size(); ++i) {
    auto& block = result.mBlocks[i];
    block.mSize = mBlockDimensions;
    block.mPose = results[i].mPose;
  }

  return result;
}
