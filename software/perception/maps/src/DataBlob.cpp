#include "DataBlob.hpp"

#include <limits>
#include <zlib.h>

using namespace maps;

DataBlob::
DataBlob() {
  mSpec.mDimensions.resize(1);
  mSpec.mDimensions[0] = 0;
  mSpec.mStrideBytes.resize(1);
  mSpec.mStrideBytes[0] = 1;
  mSpec.mCompressionType = CompressionTypeNone;
  mSpec.mDataType = DataTypeUint8;
}

DataBlob::
~DataBlob() {
}

bool DataBlob::
setData(const uint8_t* iBytes, const int iNumBytes, const Spec& iSpec) {
  mBytes.resize(iNumBytes);
  std::copy(iBytes, iBytes+iNumBytes, mBytes.begin());
  mSpec = iSpec;
  return true;
}

bool DataBlob::
setData(const std::vector<uint8_t>& iBytes, const Spec& iSpec) {
  mBytes = iBytes;
  mSpec = iSpec;
  return true;
}

DataBlob::Spec DataBlob::
getSpec() const {
  return mSpec;
}

const std::vector<uint8_t>& DataBlob::
getBytes() const {
  return mBytes;
}


bool DataBlob::
convertTo(const CompressionType& iCompressionType,
          const DataType& iDataType) {

  // specs are identical, so do nothing
  if ((mSpec.mCompressionType == iCompressionType) &&
      (mSpec.mDataType == iDataType)) {
    return true;
  }

  // data types same, intend to decompress to raw, so just decompress
  if ((iCompressionType == CompressionTypeNone) &&
      (mSpec.mDataType == iDataType)) {
    return decompress();
  }

  // data types same, intend to compress from raw, so just compress
  if ((mSpec.mCompressionType == CompressionTypeNone) &&
      (mSpec.mDataType == iDataType)) {
    return compress(iCompressionType);
  }

  // data types differ, not changing compression, so just convert type
  if ((iCompressionType == CompressionTypeNone) &&
      (mSpec.mCompressionType == CompressionTypeNone) &&
      (mSpec.mDataType != iDataType)) {
    if (convertDataType(iDataType)) {
      mSpec.mDataType = iDataType;
      return true;
    }
    return false;
  }

  // all other cases can be reduced to the following three steps.
  // some may be no-ops, but they will be caught by first base case.

  // (1) decompress if necessary, keeping same data type
  if (!convertTo(CompressionTypeNone, mSpec.mDataType)) {
    return false;
  }

  // (2) convert data types if necessary
  if (!convertTo(CompressionTypeNone, iDataType)) {
    return false;
  }

  // (3) re-compress if necessary, keeping same data type
  if (!convertTo(iCompressionType, mSpec.mDataType)) {
    return false;
  }

  return true;
}

bool DataBlob::
compress(const CompressionType& iType) {
  // if data already compressed with requested type, do nothing
  if (mSpec.mCompressionType == iType) {
    return true;
  }

  // zlib compression
  if (iType == CompressionTypeZlib) {
    std::vector<uint8_t> compressedBytes(mBytes.size()*1.001 + 12);
    unsigned long compressedSize = compressedBytes.size();
    int ret = compress2(&compressedBytes[0], &compressedSize,
                        (const Bytef*)(&mBytes[0]), mBytes.size(),
                        Z_BEST_COMPRESSION);
    if (ret != Z_OK) {
      return false;
    }
    mBytes.resize(compressedSize);
    std::copy(compressedBytes.begin(), compressedBytes.begin()+compressedSize,
              mBytes.begin());

    mSpec.mCompressionType = iType;
    return true;
  }

  return false;
}

bool DataBlob::
decompress() {
  // if already decompressed, do nothing
  if (mSpec.mCompressionType == CompressionTypeNone) {
    return true;
  }

  // zlib decompression
  if (mSpec.mCompressionType == CompressionTypeZlib) {
    std::vector<uint8_t> origBytes = mBytes;
    int totalBytes = mSpec.mStrideBytes.back()*mSpec.mDimensions.back();
    mBytes.resize(totalBytes);
    unsigned long uncompressedSize = mBytes.size();
    int ret = uncompress(&mBytes[0], &uncompressedSize,
                         &origBytes[0], origBytes.size());
    if (ret != Z_OK) {
      return false;
    }

    mSpec.mCompressionType = CompressionTypeNone;
    return true;
  }

  return false;
}

bool DataBlob::
convertDataType(const DataType& iType) {
  // can only convert if data is not compressed
  if (mSpec.mCompressionType != CompressionTypeNone) {
    return false;
  }

  // do nothing if requested type is same as current type
  if (mSpec.mDataType == iType) {
    return true;
  }

  // dispatch according to destination type
  switch (iType) {
  case DataTypeUint8:
    return convertDataTypeDispatch<uint8_t>();
  case DataTypeUint16:
    return convertDataTypeDispatch<uint16_t>();
  case DataTypeInt32:
    return convertDataTypeDispatch<int32_t>();
  case DataTypeFloat32:
    return convertDataTypeDispatch<float>();
  case DataTypeFloat64:
    return convertDataTypeDispatch<double>();
  default:
    return false;
  }
}

template<typename DestType>
bool DataBlob::
convertDataTypeDispatch() {
  switch (mSpec.mDataType) {
  case DataTypeUint8:
    return convertDataType<uint8_t,DestType>();
  case DataTypeUint16:
    return convertDataType<uint16_t,DestType>();
  case DataTypeInt32:
    return convertDataType<int32_t,DestType>();
  case DataTypeFloat32:
    return convertDataType<float,DestType>();
  case DataTypeFloat64:
    return convertDataType<double,DestType>();
  default:
    return false;
  }
}

template<typename SrcType, typename DestType>
bool DataBlob::
convertDataType() {
  // copy current data to be converted
  std::vector<uint8_t> origBytes = mBytes;

  // compute total number of elements in new array
  int totalElements = 1;
  for (int i = 0; i < mSpec.mDimensions.size(); ++i) {
    totalElements *= mSpec.mDimensions[i];
  }

  // compute new strides; data is fully packed
  std::vector<int> stridesNew = mSpec.mStrideBytes;
  stridesNew[0] = sizeof(DestType);
  for (int i = 1; i < mSpec.mDimensions.size(); ++i) {
    stridesNew[i] = mSpec.mDimensions[i-1]*stridesNew[i-1];
  }

  // copy data
  mBytes.resize(totalElements*sizeof(DestType));
  convertDataTypeIterate((SrcType*)(&origBytes[0]),
                         (DestType*)(&mBytes[0]),
                         stridesNew,
                         mSpec.mDimensions.size()-1);

  // reset byte strides
  mSpec.mStrideBytes = stridesNew;

  return true;
}

template<typename SrcType, typename DestType>
void DataBlob::
convertDataTypeIterate(SrcType* iSrcPtr, DestType* iDestPtr,
                       const std::vector<int> iStrides, const int iDim) {
  // data pointers
  SrcType* inPtr = iSrcPtr;
  DestType* outPtr = iDestPtr;

  // set clamp limits
  DestType maxVal = std::numeric_limits<DestType>::max();
  DestType minVal = std::numeric_limits<DestType>::min();
  if (!std::numeric_limits<DestType>::is_integer) {
    minVal = -maxVal;
  }

  // stride in bytes for each iteration at this level
  int strideBytes = iStrides[iDim];

  // loop at this level
  for (int i = 0; i < mSpec.mDimensions[iDim]; ++i) {

    // if this is not the individual element level, recurse
    if (iDim > 0) {
      convertDataTypeIterate(inPtr, outPtr, iStrides, iDim-1);
    }

    // if this is the individual element level, clamp and copy
    else {
      SrcType val = *inPtr;
      if (val <= minVal) {
        val = minVal;
      }
      else if (val >= maxVal) {
        val = maxVal;
      }
      *outPtr = val;
    }

    // seek to next data chunk
    inPtr = (SrcType*)((uint8_t*)inPtr + mSpec.mStrideBytes[iDim]);
    outPtr = (DestType*)((uint8_t*)outPtr + strideBytes);
  }
}
