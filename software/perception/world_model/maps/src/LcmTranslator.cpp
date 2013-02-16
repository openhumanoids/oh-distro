#include "LcmTranslator.hpp"

#include "DataBlob.hpp"
#include <octomap/octomap.h>

#include <pcl/common/transforms.h>

using namespace maps;

drc::map_params_t LcmTranslator::
toLcm(const LocalMap::Spec& iSpec) {
  drc::map_params_t msg;
  msg.map_id = iSpec.mId;
  msg.resolution = iSpec.mResolution;
  for (int i = 0; i < 3; ++i) {
    msg.bound_min[i] = iSpec.mBoundMin[i];
    msg.bound_max[i] = iSpec.mBoundMax[i];
  }
  msg.buffer_size = iSpec.mPointBufferSize;
  return msg;
}

LocalMap::Spec LcmTranslator::
fromLcm(const drc::map_params_t& iMessage) {
  LocalMap::Spec spec;
  spec.mId = iMessage.map_id;
  spec.mResolution = iMessage.resolution;
  for (int i = 0; i < 3; ++i) {
    spec.mBoundMin[i] = iMessage.bound_min[i];
    spec.mBoundMax[i] = iMessage.bound_max[i];
  }
  spec.mPointBufferSize = iMessage.buffer_size;
  spec.mActive = true;
  return spec;
}

drc::map_request_t LcmTranslator::
toLcm(const MapView::Spec& iSpec) {
  drc::map_request_t msg;
  msg.map_id = iSpec.mMapId;
  msg.view_id = iSpec.mViewId;
  msg.active = iSpec.mActive;
  msg.relative_time = iSpec.mRelativeTime;
  msg.relative_location = iSpec.mRelativeLocation;
  switch (iSpec.mType) {
  case MapView::Spec::TypeOctree:
    msg.type = drc::map_request_t::OCTREE;
    break;
  case MapView::Spec::TypeCloud:
    msg.type = drc::map_request_t::CLOUD;
    break;
  default:
    std::cout << "LcmTranslator: bad type given in map spec" << std::endl;
    break;
  }
  msg.resolution = iSpec.mResolution;
  msg.frequency = iSpec.mFrequency;
  msg.time_min = iSpec.mTimeMin;
  msg.time_max = iSpec.mTimeMax;
  msg.num_clip_planes = iSpec.mClipPlanes.size();
  msg.clip_planes.resize(msg.num_clip_planes);
  for (int i = 0; i < msg.num_clip_planes; ++i) {
    msg.clip_planes[i].resize(4);
    for (int j = 0; j < 4; ++j) {
      msg.clip_planes[i][j] = iSpec.mClipPlanes[i][j];
    }
  }
  return msg;
}

MapView::Spec LcmTranslator::
fromLcm(const drc::map_request_t& iMessage) {
  MapView::Spec spec;
  spec.mMapId = iMessage.map_id;
  spec.mViewId = iMessage.view_id;
  spec.mActive = iMessage.active;
  spec.mRelativeTime = iMessage.relative_time;
  spec.mRelativeLocation = iMessage.relative_location;
  switch (iMessage.type) {
  case drc::map_request_t::OCTREE:
    spec.mType = MapView::Spec::TypeOctree;
    break;
  case drc::map_request_t::CLOUD:
    spec.mType = MapView::Spec::TypeCloud;
    break;
  default:
    std::cout << "LcmTranslator: bad type given in map_request" << std::endl;
    break;
  }
  spec.mResolution = iMessage.resolution;
  spec.mFrequency = iMessage.frequency;
  spec.mTimeMin = iMessage.time_min;
  spec.mTimeMax = iMessage.time_max;
  spec.mClipPlanes.resize(iMessage.num_clip_planes);
  for (int i = 0; i < spec.mClipPlanes.size(); ++i) {
    for (int j = 0; j < 4; ++j) {
      spec.mClipPlanes[i][j] = iMessage.clip_planes[i][j];
    }
  }
  return spec;
}

drc::map_cloud_t LcmTranslator::
toLcm(const maps::PointCloud& iCloud) {
  drc::map_cloud_t msg;

  // find extrema of cloud and transform points
  maps::PointCloud::PointType maxPoint, minPoint;
  pcl::getMinMax3D(iCloud, minPoint, maxPoint);
  Eigen::Vector3f offset(minPoint.x, minPoint.y, minPoint.z);
  Eigen::Vector3f scale(maxPoint.x-minPoint.x, maxPoint.y-minPoint.y,
                        maxPoint.z-minPoint.z);
  scale /= 255;
  Eigen::Affine3f xform = Eigen::Affine3f::Identity();
  for (int i = 0; i < 3; ++i) {
    xform(i,i) = scale[i];
    xform(i,3) = offset[i];
  }
  xform = xform.inverse();
  maps::PointCloud cloud;
  pcl::transformPointCloud(iCloud, cloud, xform);

  // store to blob
  int totalSize = cloud.points.size()*3;
  std::vector<uint8_t> data(totalSize*sizeof(float));
  float* ptr = (float*)(&data[0]);
  for (int i = 0; i < cloud.size(); ++i) {
    maps::PointCloud::PointType pt = cloud.points[i];
    ptr[i*3+0] = pt.x;
    ptr[i*3+1] = pt.y;
    ptr[i*3+2] = pt.z;
  }
  DataBlob::Spec spec;
  spec.mDimensions.push_back(3);
  spec.mDimensions.push_back(cloud.size());
  spec.mStrideBytes.push_back(sizeof(float));
  spec.mStrideBytes.push_back(3*sizeof(float));
  spec.mCompressionType = DataBlob::CompressionTypeNone;
  spec.mDataType = DataBlob::DataTypeFloat32;
  DataBlob blob;
  blob.setData(data, spec);

  // compress and convert
  blob.convertTo(DataBlob::CompressionTypeZlib,
                 DataBlob::DataTypeUint8);

  // pack blob into message
  // NOTE: map_id and view_id not set
  msg.blob.num_dims = 2;
  msg.blob.dimensions.resize(2);
  msg.blob.stride_bytes.resize(2);
  msg.blob.dimensions[0] = spec.mDimensions[0];
  msg.blob.dimensions[1] = spec.mDimensions[1];
  msg.blob.stride_bytes[0] = spec.mStrideBytes[0];
  msg.blob.stride_bytes[1] = spec.mStrideBytes[1];
  msg.blob.compression = drc::map_blob_t::ZLIB;
  msg.blob.data_type = drc::map_blob_t::UINT8;
  msg.blob.num_bytes = data.size();
  msg.blob.data = data;
  Eigen::Affine3f xformInv = xform.inverse();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      msg.transform[i][j] = xformInv(i,j);
    }
  }

  // done
  return msg;
}

maps::PointCloud::Ptr LcmTranslator::
fromLcm(const drc::map_cloud_t& iMessage) {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());

  // transform from cloud to reference coords
  Eigen::Affine3f matx;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      matx(i,j) = iMessage.transform[i][j];
    }
  }

  // data blob
  DataBlob::Spec spec;
  const drc::map_blob_t& msgBlob = iMessage.blob;
  spec.mDimensions.resize(msgBlob.dimensions.size());
  std::copy(msgBlob.dimensions.begin(), msgBlob.dimensions.end(),
            spec.mDimensions.begin());
  spec.mStrideBytes.resize(msgBlob.stride_bytes.size());
  std::copy(msgBlob.stride_bytes.begin(), msgBlob.stride_bytes.end(),
            spec.mStrideBytes.begin());
  switch (msgBlob.compression) {
  case drc::map_blob_t::UNCOMPRESSED:
    spec.mCompressionType = maps::DataBlob::CompressionTypeNone;
    break;
  case drc::map_blob_t::ZLIB:
    spec.mCompressionType = maps::DataBlob::CompressionTypeZlib;
    break;
  default:
    std::cout << "LcmTranslator: bad compression type in cloud" << std::endl;
    break;
  }
  switch (msgBlob.data_type) {
  case drc::map_blob_t::UINT8:
    spec.mDataType = maps::DataBlob::DataTypeUint8;
    break;
  case drc::map_blob_t::UINT16:
    spec.mDataType = maps::DataBlob::DataTypeUint16;
    break;
  case drc::map_blob_t::FLOAT32:
    spec.mDataType = maps::DataBlob::DataTypeFloat32;
    break;
  default:
    std::cout << "LcmTranslator: bad data type in cloud" << std::endl;
    break;
  }
  DataBlob blob;
  blob.setData(msgBlob.data, spec);

  // convert to point cloud
  blob.convertTo(maps::DataBlob::CompressionTypeNone,
                 maps::DataBlob::DataTypeFloat32);
  float* raw = (float*)(&blob.getBytes()[0]);
  cloud->resize(spec.mDimensions[1]);
  for (size_t i = 0; i < cloud->size(); ++i) {
    (*cloud)[i].x = raw[i*3 + 0];
    (*cloud)[i].y = raw[i*3 + 1];
    (*cloud)[i].z = raw[i*3 + 2];
  }    

  // transform and return
  pcl::transformPointCloud(*cloud, *cloud, matx);
  return cloud;
}

drc::map_octree_t LcmTranslator::
toLcm(const maps::Octree& iTree) {
  // NOTE: map_id and view_id not set here
  drc::map_octree_t msg;
  std::ostringstream oss;
  iTree.mTree->writeBinaryConst(oss);
  std::string str = oss.str();
  Eigen::Isometry3f matx = iTree.mTransform.inverse();
  msg.num_bytes = str.size();
  msg.data.resize(msg.num_bytes);
  std::copy(str.begin(), str.end(), msg.data.begin());
  Eigen::Quaternionf q(matx.linear());
  Eigen::Vector3f t(matx.translation());
  msg.transform.translation.x = t[0];
  msg.transform.translation.y = t[1];
  msg.transform.translation.z = t[2];
  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  return msg;
}

maps::Octree LcmTranslator::
fromLcm(const drc::map_octree_t& iMessage) {
  maps::Octree octree;
  Eigen::Quaternionf q(iMessage.transform.rotation.w,
                       iMessage.transform.rotation.x,
                       iMessage.transform.rotation.y,
                       iMessage.transform.rotation.z);
  Eigen::Vector3f pos(iMessage.transform.translation.x,
                      iMessage.transform.translation.y,
                      iMessage.transform.translation.z);
  octree.mTransform.linear() = q.matrix();
  octree.mTransform.translation() = pos;
  octree.mTransform = octree.mTransform.inverse();
  std::string str(iMessage.data.begin(), iMessage.data.end());
  std::stringstream ss(str);
  octree.mTree.reset(new octomap::OcTree(0.1));
  octree.mTree->readBinary(ss);
  return octree;
}
