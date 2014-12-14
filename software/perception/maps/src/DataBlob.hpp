#ifndef _maps_DataBlob_hpp_
#define _maps_DataBlob_hpp_

#include <inttypes.h>
#include <vector>

namespace maps {

class DataBlob {
public:
  enum CompressionType {
    CompressionTypeNone,
    CompressionTypeZlib
  };

  enum DataType {
    DataTypeUint8,
    DataTypeUint16,
    DataTypeInt32,
    DataTypeFloat32,
    DataTypeFloat64
  };

  struct Spec {
    std::vector<int> mDimensions;      // size in w, h, d, etc
    std::vector<int> mStrideBytes;     // stride in w, h, d, etc
    CompressionType mCompressionType;
    DataType mDataType;
  };

public:
  DataBlob();
  virtual ~DataBlob();

  bool setData(const uint8_t* iBytes, const int iNumBytes, const Spec& iSpec);
  bool setData(const std::vector<uint8_t>& iBytes, const Spec& iSpec);
  bool convertTo(const CompressionType& iCompressionType,
                 const DataType& iDataType);

  Spec getSpec() const;
  const std::vector<uint8_t>& getBytes() const;

protected:
  bool compress(const CompressionType& iType);
  bool decompress();

  bool convertDataType(const DataType& iType);
  template<typename DestType>
  bool convertDataTypeDispatch();
  template<typename SrcType, typename DestType>
  bool convertDataType();
  template<typename SrcType, typename DestType>
  void convertDataTypeIterate(SrcType* iSrcPtr, DestType* iDestPtr,
                              const std::vector<int> iStrides, const int iDim);

protected:
  Spec mSpec;
  std::vector<uint8_t> mBytes;
};

}

#endif
