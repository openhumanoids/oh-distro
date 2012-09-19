#ifndef _ThreadSafeQueue_hpp_
#define _ThreadSafeQueue_hpp_

#include <deque>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

template <typename T>
class ThreadSafeQueue {

public:

  ThreadSafeQueue() {
    setMaxSize(-1);
  }

  ~ThreadSafeQueue() {
  }

  void setMaxSize(const int iSize) {
    mMaxSize = iSize;
    boost::mutex::scoped_lock lock(mMutex);
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
    boost::mutex::scoped_lock lock(mMutex);
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
    boost::mutex::scoped_lock lock(mMutex);
    if (mData.size() == 0) {
      return false;
    }
    oData = mData.front();
    mData.pop_front();
    return true;
  }

  bool waitForData(T& oData) {
    boost::mutex::scoped_lock lock(mMutex);
    while (mData.empty()) {
      mCondition.wait(lock);
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
    mCondition.notify_all();
  }

protected:
  int mMaxSize;
  std::deque<T> mData;
  boost::mutex mMutex;
  boost::condition_variable mCondition;
};

#endif
