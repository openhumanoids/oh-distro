#ifndef _drc_ThreadSafeQueue_hpp_
#define _drc_ThreadSafeQueue_hpp_

#include <deque>
#include <thread>

namespace drc {

template <typename T>
class ThreadSafeQueue {

public:

  ThreadSafeQueue() {
    setMaxSize(-1);
    mUnblock = false;
  }

  ~ThreadSafeQueue() {
    unblock();
  }

  void setMaxSize(const int iSize) {
    mMaxSize = iSize;
    std::lock_guard<std::mutex> lock(mMutex);
    if (mMaxSize >= 0) {
      while (mData.size() > mMaxSize) {
        mData.pop_front();
      }
    }
  }

  int getSize() const {
    return mData.size();
  }

  void push(const T& iData) {
    std::unique_lock<std::mutex> lock(mMutex);
    if (mMaxSize >= 0) {
      while (mData.size() >= mMaxSize) {
        mData.pop_front();
      }
    }
    mData.push_back(iData);
    lock.unlock();
    mCondition.notify_one();
  }    

  bool pop(T& oData) {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mData.size() == 0) {
      return false;
    }
    oData = mData.front();
    mData.pop_front();
    return true;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mMutex);
    mData.clear();
    unblock();
  }

  bool waitForData(T& oData) {
    std::unique_lock<std::mutex> lock(mMutex);
    while (!mUnblock && mData.empty()) {
      mCondition.wait(lock);
    }
    if (mUnblock) {
      mUnblock = false;
    }
    if (!mData.empty()) {
      oData = mData.front();
      mData.pop_front();
      return true;
    }
    else {
      return false;
    }
  }

  void unblock() {
    mUnblock = true;
    mCondition.notify_all();
  }

protected:
  int mMaxSize;
  std::deque<T> mData;
  std::mutex mMutex;
  std::condition_variable mCondition;
  bool mUnblock;
};

}

#endif
