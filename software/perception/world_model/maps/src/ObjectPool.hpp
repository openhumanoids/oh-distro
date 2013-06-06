#ifndef _maps_ObjectPool_hpp_
#define _maps_ObjectPool_hpp_

#include <vector>
#include <memory>

namespace maps {

template<typename T, int N>
class ObjectPool {
public:
  typedef std::shared_ptr<T> DataType;

private:
  struct Object {
    bool mInUse;
    DataType mData;
  };

public:

  ObjectPool() {
    mObjects.resize(N);
    for (size_t i = 0; i < mObjects.size(); ++i) {
      mObjects[i].mInUse = false;
      mObjects[i].mData.reset(new T());  // TODO: what about constructor arguments?
    }
  }

  DataType get() {
    for (size_t i = 0; i < mObjects.size(); ++i) {
      if (mObjects[i].mInUse && (mObjects[i].mData.use_count() == 1)) {
        mObjects[i].mInUse = false;
      }
      if (!mObjects[i].mInUse) {
        mObjects[i].mInUse = true;
        return mObjects[i].mData;
      }
    }
    return DataType();
  }

  bool done(const DataType& iData) {
    for (size_t i = 0; i < mObjects.size(); ++i) {
      if (mObjects[i].mInUse && (mObjects[i].mData == iData)) {
        mObjects[i].mInUse = false;
        return true;
      }
    }
    return false;
  }

  int getNumFree() {
    int total = 0;
    for (size_t i = 0; i < mObjects.size(); ++i) {
      if (mObjects[i].mData.use_count() == 1) mObjects[i].mInUse = false;
      if (!mObjects[i].mInUse) ++total;
    }
    return total;
  }

  int getCapacity() const {
    return mObjects.size();
  }

protected:
  std::vector<Object> mObjects;
};

}

#endif
