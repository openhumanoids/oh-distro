#include "LcmTranslator.hpp"

#include "DataBlob.hpp"
#include "DepthImageView.hpp"
#include "DepthImage.hpp"

#include <iostream>
#include <limits>

#include <lcmtypes/drc/map_request_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>

using namespace maps;


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
  for (int i = 0; i < (int)oSpec.mClipPlanes.size(); ++i) {
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
    zScale /= ((1 << bits) - 1);
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
  case DataBlob::DataTypeUint8: maxVal = 254; break;
  case DataBlob::DataTypeUint16: maxVal = 65534; break;
  default: maxVal = std::numeric_limits<float>::infinity(); break;
  }
  blob.convertTo(DataBlob::CompressionTypeNone, DataBlob::DataTypeFloat32); 
  float* raw = (float*)(&blob.getBytes()[0]);
  std::vector<float> depths(raw, raw+blob.getBytes().size()/sizeof(float));
  for (int i = 0; i < (int)depths.size(); ++i) {
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
