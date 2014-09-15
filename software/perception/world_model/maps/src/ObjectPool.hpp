#ifndef _maps_ObjectPool_hpp_
#define _maps_ObjectPool_hpp_

#include <vector>
#include <memory>

namespace maps {

template<typename T, int N>
class ObjectPool {
public:
  typedef std::shared_ptr<T> DataType;

public:

  ObjectPool() {
    mObjects.resize(N);
    for (size_t i = 0; i < mObjects.size(); ++i) {
      mObjects[i].reset(new T());  // TODO: what about constructor arguments?
    }
  }

  DataType get() {
    for (size_t i = 0; i < mObjects.size(); ++i) {
      if (mObjects[i].unique()) {
        return mObjects[i];
      }
    }
    return DataType();
  }

  int getNumFree() {
    int total = 0;
    for (size_t i = 0; i < mObjects.size(); ++i) {
      if (mObjects[i].unique()) ++total;
    }
    return total;
  }

  int getCapacity() const {
    return mObjects.size();
  }

protected:
  std::vector<DataType> mObjects;
};

}

#endif
