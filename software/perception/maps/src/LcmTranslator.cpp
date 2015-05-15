#include "LcmTranslator.hpp"

#include "DataBlob.hpp"
#include "PointCloudView.hpp"
#include "OctreeView.hpp"
#include "DepthImageView.hpp"
#include "DepthImage.hpp"
#include "ScanBundleView.hpp"
#include "Utils.hpp"

#include <limits>
#include <octomap/octomap.h>

#include <lcmtypes/drc/map_params_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_scans_t.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace maps;

namespace {
  void writeDepths(const DepthImageView& iView, const std::string& iFileName) {
    DepthImage::Type imageType = DepthImage::TypeDepth;
    DepthImage::Ptr depthImage = iView.getDepthImage();
    std::vector<float> inDepths = depthImage->getData(imageType);
    int width = depthImage->getWidth();
    int height = depthImage->getHeight();

    std::ofstream ofs(iFileName.c_str());
    ofs.write((char*)(&width), sizeof(int));
    ofs.write((char*)(&height), sizeof(int));
    ofs.write((char*)inDepths.data(), inDepths.size()*sizeof(float));
    ofs.close();
  }
}

bool LcmTranslator::
toLcm(const LocalMap::Spec& iSpec, drc::map_params_t& oMessage) {
  oMessage.map_id = iSpec.mId;
  oMessage.resolution = iSpec.mResolution;
  oMessage.buffer_size = iSpec.mPointBufferSize;
  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_params_t& iMessage, LocalMap::Spec& oSpec) {
  oSpec.mId = iMessage.map_id;
  oSpec.mResolution = iMessage.resolution;
  oSpec.mPointBufferSize = iMessage.buffer_size;
  oSpec.mActive = true;
  return true;
}

bool LcmTranslator::
toLcm(const ViewBase::Spec& iSpec, drc::map_request_t& oMessage) {
  oMessage.map_id = iSpec.mMapId;
  oMessage.view_id = iSpec.mViewId;
  oMessage.active = iSpec.mActive;
  oMessage.time_mode = iSpec.mTimeMode;
  oMessage.relative_location = iSpec.mRelativeLocation;
  oMessage.accum_type = iSpec.mAccumulationMethod;
  switch (iSpec.mType) {
  case ViewBase::TypeOctree:
    oMessage.type = drc::map_request_t::OCTREE; break;
  case ViewBase::TypePointCloud:
    oMessage.type = drc::map_request_t::POINT_CLOUD; break;
  case ViewBase::TypeDepthImage:
    oMessage.type = drc::map_request_t::DEPTH_IMAGE; break;
  case ViewBase::TypeScanBundle:
    oMessage.type = drc::map_request_t::SCAN_BUNDLE; break;
  default:
    std::cout << "LcmTranslator: bad type given in map spec" << std::endl;
    return false;
  }
  oMessage.resolution = iSpec.mResolution;
  oMessage.frequency = iSpec.mFrequency;
  oMessage.quantization_max = iSpec.mQuantizationMax;
  oMessage.channel = iSpec.mChannel;
  oMessage.time_min = iSpec.mTimeMin;
  oMessage.time_max = iSpec.mTimeMax;
  oMessage.width = iSpec.mWidth;
  oMessage.height = iSpec.mHeight;
  oMessage.num_clip_planes = iSpec.mClipPlanes.size();
  oMessage.clip_planes.resize(oMessage.num_clip_planes);
  for (int i = 0; i < oMessage.num_clip_planes; ++i) {
    oMessage.clip_planes[i].resize(4);
    for (int j = 0; j < 4; ++j) {
      oMessage.clip_planes[i][j] = iSpec.mClipPlanes[i][j];
    }
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oMessage.transform[i][j] = iSpec.mTransform(i,j);
    }
  }
  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_request_t& iMessage, ViewBase::Spec& oSpec) {
  oSpec.mMapId = iMessage.map_id;
  oSpec.mViewId = iMessage.view_id;
  oSpec.mActive = iMessage.active;
  oSpec.mTimeMode = (ViewBase::TimeMode)iMessage.time_mode;
  oSpec.mRelativeLocation = iMessage.relative_location;
  oSpec.mAccumulationMethod = (ViewBase::AccumulationMethod)iMessage.accum_type;
  switch (iMessage.type) {
  case drc::map_request_t::OCTREE:
    oSpec.mType = ViewBase::TypeOctree; break;
  case drc::map_request_t::POINT_CLOUD:
    oSpec.mType = ViewBase::TypePointCloud; break;
  case drc::map_request_t::DEPTH_IMAGE:
    oSpec.mType = ViewBase::TypeDepthImage; break;
  case drc::map_request_t::SCAN_BUNDLE:
    oSpec.mType = ViewBase::TypeScanBundle; break;
  default:
    std::cout << "LcmTranslator: bad type given in map_request" << std::endl;
    break;
  }
  oSpec.mResolution = iMessage.resolution;
  oSpec.mFrequency = iMessage.frequency;
  oSpec.mQuantizationMax = iMessage.quantization_max;
  oSpec.mChannel = iMessage.channel;
  oSpec.mTimeMin = iMessage.time_min;
  oSpec.mTimeMax = iMessage.time_max;
  oSpec.mWidth = iMessage.width;
  oSpec.mHeight = iMessage.height;
  oSpec.mClipPlanes.resize(iMessage.num_clip_planes);
  for (int i = 0; i < oSpec.mClipPlanes.size(); ++i) {
    for (int j = 0; j < 4; ++j) {
      oSpec.mClipPlanes[i][j] = iMessage.clip_planes[i][j];
    }
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oSpec.mTransform(i,j) = iMessage.transform[i][j];
    }
  }
  return true;
}

bool LcmTranslator::
toLcm(const maps::DataBlob& iBlob, drc::map_blob_t& oMessage) {
  DataBlob::Spec spec = iBlob.getSpec();
  oMessage.num_dims = spec.mDimensions.size();
  oMessage.dimensions.resize(oMessage.num_dims);
  oMessage.stride_bytes.resize(oMessage.num_dims);
  std::copy(spec.mDimensions.begin(), spec.mDimensions.end(),
            oMessage.dimensions.begin());
  std::copy(spec.mStrideBytes.begin(), spec.mStrideBytes.end(),
            oMessage.stride_bytes.begin());
  switch (spec.mCompressionType) {
  case DataBlob::CompressionTypeNone:
    oMessage.compression = drc::map_blob_t::UNCOMPRESSED; break;
  case DataBlob::CompressionTypeZlib:
    oMessage.compression = drc::map_blob_t::ZLIB; break;
  default:
    std::cout << "LcmTranslator: bad compression type in blob" << std::endl;
    return false;
  }
  switch(spec.mDataType) {
  case DataBlob::DataTypeUint8:
    oMessage.data_type = drc::map_blob_t::UINT8; break;
  case DataBlob::DataTypeUint16:
    oMessage.data_type = drc::map_blob_t::UINT16; break;
  case DataBlob::DataTypeFloat32:
    oMessage.data_type = drc::map_blob_t::FLOAT32; break;
  case DataBlob::DataTypeInt32:
  case DataBlob::DataTypeFloat64:
  default:
    std::cout << "LcmTranslator: unsupported blob data type" << std::endl;
    return false;
  }
  oMessage.num_bytes = iBlob.getBytes().size();
  oMessage.data = iBlob.getBytes();
  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_blob_t& iMessage, maps::DataBlob& oBlob) {
  DataBlob::Spec spec;
  spec.mDimensions.resize(iMessage.dimensions.size());
  std::copy(iMessage.dimensions.begin(), iMessage.dimensions.end(),
            spec.mDimensions.begin());
  spec.mStrideBytes.resize(iMessage.stride_bytes.size());
  std::copy(iMessage.stride_bytes.begin(), iMessage.stride_bytes.end(),
            spec.mStrideBytes.begin());
  switch (iMessage.compression) {
  case drc::map_blob_t::UNCOMPRESSED:
    spec.mCompressionType = maps::DataBlob::CompressionTypeNone; break;
  case drc::map_blob_t::ZLIB:
    spec.mCompressionType = maps::DataBlob::CompressionTypeZlib; break;
  default:
    std::cout << "LcmTranslator: bad compression type in cloud" << std::endl;
    return false;
  }
  switch (iMessage.data_type) {
  case drc::map_blob_t::UINT8:
    spec.mDataType = maps::DataBlob::DataTypeUint8; break;
  case drc::map_blob_t::UINT16:
    spec.mDataType = maps::DataBlob::DataTypeUint16; break;
  case drc::map_blob_t::FLOAT32:
    spec.mDataType = maps::DataBlob::DataTypeFloat32; break;
  default:
    std::cout << "LcmTranslator: bad blob data type" << std::endl;
    return false;
  }
  oBlob.setData(iMessage.data, spec);
  return true;
}


bool LcmTranslator::
toLcm(const PointCloudView& iView, drc::map_cloud_t& oMessage,
      const float iQuantMax, const bool iCompress) {
  oMessage.view_id = iView.getId();

  // find extrema of cloud and transform points
  maps::PointCloud::PointType maxPoint, minPoint;
  pcl::getMinMax3D(*(iView.getPointCloud()), minPoint, maxPoint);
  Eigen::Vector3f offset = minPoint.getVector3fMap();
  Eigen::Vector3f scale = maxPoint.getVector3fMap() - offset;
  int bits = 8;
  if (iQuantMax == 0) {
    bits = 32;
    scale << 1,1,1;
  }
  else {
    if (iQuantMax > 0) {
      bits = ceil(std::log2(scale.maxCoeff()/iQuantMax));
      bits = std::min(bits, 16);
      bits = std::max(bits, 0);
    }
    scale /= ((1 << bits) - 1);
  }
  Eigen::Affine3f normalizerInv = Eigen::Affine3f::Identity();
  for (int i = 0; i < 3; ++i) {
    normalizerInv(i,i) = scale[i];
    normalizerInv(i,3) = offset[i];
  }
  Eigen::Affine3f normalizer = normalizerInv.inverse();
  maps::PointCloud cloud;
  pcl::transformPointCloud(*(iView.getPointCloud()), cloud, normalizer);

  // store to blob
  int totalSize = cloud.points.size()*3;
  std::vector<uint8_t> data(totalSize*sizeof(float));
  float* ptr = (float*)(&data[0]);
  for (int i = 0; i < cloud.size(); ++i) {
    maps::PointCloud::PointType pt = cloud.points[i];
    ptr[i*3+0] = pt.x + 0.5f;  // add 0.5 for better rounding
    ptr[i*3+1] = pt.y + 0.5f;
    ptr[i*3+2] = pt.z + 0.5f;
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
  DataBlob::CompressionType compressionType =
    iCompress ? DataBlob::CompressionTypeZlib : DataBlob::CompressionTypeNone;
  DataBlob::DataType dataType;
  if (bits <= 8) dataType = DataBlob::DataTypeUint8;
  else if (bits <= 16) dataType = DataBlob::DataTypeUint16;
  else dataType = DataBlob::DataTypeFloat32;
  if (!blob.convertTo(compressionType, dataType)) {
    std::cout << "LcmTranslator: cannot compress data" << std::endl;
    return false;
  }

  // pack blob into message
  if (!toLcm(blob, oMessage.blob)) return false;

  // transform
  Eigen::Projective3f fromRef = normalizer*iView.getTransform();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oMessage.transform[i][j] = fromRef(i,j);
    }
  }

  // done
  // NOTE: map_id not set
  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_cloud_t& iMessage, PointCloudView& oView) {

  // transform from reference to cloud coords
  Eigen::Projective3f xform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      xform(i,j) = iMessage.transform[i][j];
    }
  }
  oView.setId(iMessage.view_id);
  oView.setTransform(xform);

  // data blob
  DataBlob blob;
  if (!fromLcm(iMessage.blob, blob)) return false;

  // convert to point cloud
  if (!blob.convertTo(DataBlob::CompressionTypeNone,
                      DataBlob::DataTypeFloat32)) {
    std::cout << "LcmTranslator: could not decompress data" << std::endl;
    return false;
  }
  float* raw = (float*)(&blob.getBytes()[0]);
  maps::PointCloud& cloud = *(oView.getPointCloud());
  cloud.resize(blob.getSpec().mDimensions[1]);
  for (size_t i = 0; i < cloud.size(); ++i) {
    cloud[i].x = raw[i*3 + 0];
    cloud[i].y = raw[i*3 + 1];
    cloud[i].z = raw[i*3 + 2];
  }    

  return true;
}

bool LcmTranslator::
toLcm(const OctreeView& iView, drc::map_octree_t& oMessage) {
  // NOTE: map_id not set here
  oMessage.view_id = iView.getId();
  std::ostringstream oss;
  iView.getOctree()->writeBinaryConst(oss);
  std::string str = oss.str();
  Eigen::Projective3f xform = iView.getTransform();
  oMessage.num_bytes = str.size();
  oMessage.data.resize(oMessage.num_bytes);
  std::copy(str.begin(), str.end(), oMessage.data.begin());
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oMessage.transform[i][j] = xform(i,j);
    }
  }
  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_octree_t& iMessage, OctreeView& oView) {
  Eigen::Projective3f xform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      xform(i,j) = iMessage.transform[i][j];
    }
  }
  oView.setId(iMessage.view_id);
  oView.setTransform(xform);
  std::string str(iMessage.data.begin(), iMessage.data.end());
  std::stringstream ss(str);
  oView.getOctree()->readBinary(ss);
  return true;
}

bool LcmTranslator::
toLcm(const DepthImageView& iView, drc::map_image_t& oMessage,
      const float iQuantMax, const bool iCompress) {

  oMessage.view_id = iView.getId();

  // copy depth array
  DepthImage::Type imageType = DepthImage::TypeDepth;
  DepthImage::Ptr depthImage = iView.getDepthImage();
  float invalidValue = std::numeric_limits<float>::infinity();
  int numDepths = depthImage->getWidth() * depthImage->getHeight();
  std::vector<float> inDepths = depthImage->getData(imageType);
  std::vector<uint8_t> data((uint8_t*)(&inDepths[0]),
                            (uint8_t*)(&inDepths[0]) + numDepths*sizeof(float));
  float* outDepths = (float*)(&data[0]);

  // find extremal values and transform
  float zMin(std::numeric_limits<float>::infinity());
  float zMax(-std::numeric_limits<float>::infinity());
  for (int i = 0; i < numDepths; ++i) {
    float val = outDepths[i];
    if (val == invalidValue) continue;
    zMin = std::min(zMin, val);
    zMax = std::max(zMax, val);
  }
  if (zMin >= zMax) {
    zMin = 0;
    zMax = 1;
  }
  float zOffset(zMin), zScale(zMax-zMin);
  int bits = 11;
  if (iQuantMax == 0) {
    bits = 32;
    zScale = 1;
    zOffset = 0;
  }
  else {
    if (iQuantMax > 0) {
      bits = ceil(std::log2(zScale/iQuantMax));
      bits = std::min(bits, 16);
      bits = std::max(bits, 0);
    }
    zScale /= ((1 << bits) - 2);  // leave room for max int value (for invalid)
  }
  for (int i = 0; i < numDepths; ++i) {
    float val = outDepths[i];
    if (val == invalidValue) continue;
    outDepths[i] = (outDepths[i]-zOffset)/zScale;
    if (bits < 32) outDepths[i] += 0.5f;  // for rounding
  }

  // store to blob
  DataBlob::Spec spec;
  spec.mDimensions.push_back(depthImage->getWidth());
  spec.mDimensions.push_back(depthImage->getHeight());
  spec.mStrideBytes.push_back(sizeof(float));
  spec.mStrideBytes.push_back(depthImage->getWidth()*sizeof(float));
  spec.mCompressionType = DataBlob::CompressionTypeNone;
  spec.mDataType = DataBlob::DataTypeFloat32;
  DataBlob blob;
  blob.setData(data, spec);

  // compress and convert
  DataBlob::CompressionType compressionType =
    iCompress ? DataBlob::CompressionTypeZlib : DataBlob::CompressionTypeNone;
  DataBlob::DataType dataType;
  if (bits <= 8) dataType = DataBlob::DataTypeUint8;
  else if (bits <= 16) dataType = DataBlob::DataTypeUint16;
  else dataType = DataBlob::DataTypeFloat32;
  blob.convertTo(compressionType, dataType);
  if (!toLcm(blob, oMessage.blob)) return false;

  // transform from reference to image
  Eigen::Projective3f xform = iView.getTransform();
  oMessage.data_scale = zScale;
  oMessage.data_shift = zOffset;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oMessage.transform[i][j] = xform(i,j);
    }
  }

  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_image_t& iMessage, DepthImageView& oView) {

  // transform from reference to image coords
  Eigen::Projective3f xform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      xform(i,j) = iMessage.transform[i][j];
    }
  }

  // data blob
  DataBlob blob;
  if (!fromLcm(iMessage.blob, blob)) return false;

  // convert to depth image
  float maxVal;
  switch(blob.getSpec().mDataType) {
  case DataBlob::DataTypeUint8: maxVal = 255; break;
  case DataBlob::DataTypeUint16: maxVal = 65535; break;
  default: maxVal = std::numeric_limits<float>::infinity(); break;
  }
  blob.convertTo(DataBlob::CompressionTypeNone, DataBlob::DataTypeFloat32); 
  float* raw = (float*)(&blob.getBytes()[0]);
  std::vector<float> depths(raw, raw+blob.getBytes().size()/sizeof(float));
  for (int i = 0; i < depths.size(); ++i) {
    if (depths[i] >= maxVal) {
      depths[i] = std::numeric_limits<float>::infinity();
    }
    else {
      depths[i] = iMessage.data_scale*depths[i] + iMessage.data_shift;
    }
  }

  DepthImage img;
  img.setSize(blob.getSpec().mDimensions[0], blob.getSpec().mDimensions[1]);
  img.setProjector(xform);
  img.setData(depths, DepthImage::TypeDepth);
  oView.setId(iMessage.view_id);
  oView.set(img);

  // done
  // NOTE: ids not set here
  return true;
}


bool LcmTranslator::
toLcm(const LidarScan& iScan, drc::map_scan_t& oMessage,
      const float iQuantMax, const bool iCompress,
      const bool iIncludeIntensities) {
  auto& msg = oMessage;

  // basic info
  msg.utime = iScan.getTimestamp();
  msg.num_ranges = iScan.getNumRanges();
  msg.theta_min = iScan.getThetaMin();
  msg.theta_step = iScan.getThetaStep();
  const Eigen::Vector3f startPos = iScan.getStartPose().translation();
  const Eigen::Vector3f endPos = iScan.getEndPose().translation();
  const Eigen::Quaternionf startQuat(iScan.getStartPose().rotation());
  const Eigen::Quaternionf endQuat(iScan.getEndPose().rotation());
  for (int k = 0; k < 3; ++k) msg.translation_start[k] = startPos[k];
  for (int k = 0; k < 3; ++k) msg.translation_end[k] = endPos[k];
  msg.quaternion_start[0] = startQuat.w();
  msg.quaternion_start[1] = startQuat.x();
  msg.quaternion_start[2] = startQuat.y();
  msg.quaternion_start[3] = startQuat.z();
  msg.quaternion_end[0] = endQuat.w();
  msg.quaternion_end[1] = endQuat.x();
  msg.quaternion_end[2] = endQuat.y();
  msg.quaternion_end[3] = endQuat.z();

  // examine ranges and determine number of bits required
  std::vector<float> ranges = iScan.getRanges();
  for (auto& r : ranges) r = std::max(r,0.0f);
  float maxRange = 0;
  for (auto& r : ranges) maxRange = std::max(r, maxRange);
  float rangeScale = maxRange;
  int bits = 11;
  if (iQuantMax == 0) {
    bits = 32;
    rangeScale = 1;
  }
  else {
    if (iQuantMax > 0) {
      bits = ceil(std::log2(rangeScale/iQuantMax));
      bits = std::max(std::min(bits, 16), 0);
    }
    rangeScale /= ((1 << bits) - 1);
  }
  msg.range_scale = rangeScale;
  if (bits == 32) {
    for (auto& r : ranges) r /= rangeScale;
  }
  else {
    for (auto& r : ranges) r = r/rangeScale + 0.5f;
  }

  // store ranges to blob
  std::vector<uint8_t> bytes((uint8_t*)ranges.data(),
                             (uint8_t*)(ranges.data() + ranges.size()));
  DataBlob::Spec spec;
  spec.mDimensions.push_back(msg.num_ranges);
  spec.mStrideBytes.push_back(sizeof(float));
  spec.mCompressionType = DataBlob::CompressionTypeNone;
  spec.mDataType = DataBlob::DataTypeFloat32;
  DataBlob blob;
  blob.setData(bytes, spec);
  DataBlob::CompressionType compressionType =
    iCompress ? DataBlob::CompressionTypeZlib : DataBlob::CompressionTypeNone;
  DataBlob::DataType dataType;
  if (bits <= 8) dataType = DataBlob::DataTypeUint8;
  else if (bits <= 16) dataType = DataBlob::DataTypeUint16;
  else dataType = DataBlob::DataTypeFloat32;
  blob.convertTo(compressionType, dataType);
  if (!toLcm(blob, oMessage.range_blob)) return false;

  // store intensities to blob
  if (iIncludeIntensities &&
      (iScan.getNumIntensities()==iScan.getNumRanges())) {
    const float intensityScale = 4000/255;   // TODO: hardcoded for now
    msg.intensity_scale = intensityScale;
    std::vector<float> intensities = iScan.getIntensities();
    for (size_t i = 0; i < ranges.size(); ++i) {
      intensities[i] = intensities[i]/intensityScale + 0.5f;
      if (ranges[i] <= 0) intensities[i] = 0;
      if (intensities[i] > 255) intensities[i] = 255;
    }
    bytes = std::vector<uint8_t>
      ((uint8_t*)intensities.data(),
       (uint8_t*)(intensities.data() + intensities.size()));
    blob.setData(bytes, spec);
    blob.convertTo(compressionType, DataBlob::DataTypeUint8);
    if (!toLcm(blob, oMessage.intensity_blob)) return false;
  }

  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_scan_t& iMessage, LidarScan& oScan) {
  const auto& msg = iMessage;

  // basic info
  oScan.setTimestamp(msg.utime);
  oScan.setAngles(msg.theta_min, msg.theta_step);

  // poses
  Eigen::Isometry3f startPose = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f endPose = Eigen::Isometry3f::Identity();
  for (int k = 0; k < 3; ++k) startPose(k,3) = msg.translation_start[k];
  for (int k = 0; k < 3; ++k) endPose(k,3) = msg.translation_end[k];
  Eigen::Quaternionf startQuat, endQuat;
  for (int k = 0; k < 4; ++k) startQuat.coeffs()[k] =
                                msg.quaternion_start[(k+1)%4];
  for (int k = 0; k < 4; ++k) endQuat.coeffs()[k] = msg.quaternion_end[(k+1)%4];
  startPose.linear() = startQuat.matrix();
  endPose.linear() = endQuat.matrix();
  oScan.setPoses(startPose, endPose);

  // ranges
  DataBlob blob;
  if (!fromLcm(msg.range_blob, blob)) return false;
  blob.convertTo(DataBlob::CompressionTypeNone, DataBlob::DataTypeFloat32);
  float* raw = (float*)(&blob.getBytes()[0]);
  std::vector<float> ranges(raw, raw+blob.getBytes().size()/sizeof(float));
  for (auto& r : ranges) r *= msg.range_scale;
  oScan.setRanges(ranges);

  // intensities
  if (fromLcm(msg.intensity_blob, blob)) {
    blob.convertTo(DataBlob::CompressionTypeNone, DataBlob::DataTypeFloat32);
    raw = (float*)blob.getBytes().data();
    std::vector<float> intensities
      (raw, raw+blob.getBytes().size()/sizeof(float));
    if (intensities.size() == ranges.size()) {
      for (auto& val : intensities) val *= msg.intensity_scale;
      oScan.setIntensities(intensities);
    }
  }

  return true;
}

bool LcmTranslator::
toLcm(const ScanBundleView& iView, drc::map_scans_t& oMessage,
      const float iQuantMax, const bool iCompress,
      const bool iIncludeIntensities) {

  const auto& scans = iView.getScans();
  const int numScans = scans.size();

  // set basic info
  oMessage.view_id = iView.getId();
  oMessage.num_scans = numScans;
  oMessage.scans.resize(numScans);
  auto xform = iView.getTransform();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) oMessage.transform[i][j] = xform(i,j);
  }
  oMessage.chunk_id = 0;
  oMessage.num_chunks = 1;

  // set scan data
  oMessage.data_bytes = 0;
  for (int i = 0; i < numScans; ++i) {
    drc::map_scan_t& msg = oMessage.scans[i];
    LidarScan::Ptr scan = scans[i];
    toLcm(*scan, msg, iQuantMax, iCompress, iIncludeIntensities);
    oMessage.data_bytes += msg.range_blob.num_bytes;
    oMessage.data_bytes += msg.intensity_blob.num_bytes;
  }

  return true;
}

bool LcmTranslator::
fromLcm(const drc::map_scans_t& iMessage, ScanBundleView& oView) {
  Eigen::Projective3f xform;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) xform(i,j) = iMessage.transform[i][j];
  }
  oView.setTransform(xform);
  const int numScans = iMessage.num_scans;
  std::vector<LidarScan::Ptr> scans(numScans);
  for (auto& scan : scans) scan.reset(new LidarScan());
  for (int i = 0; i < numScans; ++i) fromLcm(iMessage.scans[i], *scans[i]);
  oView.set(scans);
  return true;
}
