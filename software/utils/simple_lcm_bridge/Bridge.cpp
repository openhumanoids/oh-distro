#include "Bridge.hpp"

#include <iostream>
#include <unordered_map>
#include <vector>
#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/Clock.hpp>

#include <lcmtypes/drc/message_rate_t.hpp>

struct Bridge::Imp {
  struct Binding {
    drc::LcmWrapper::Ptr mDestLcmWrapper;
    BindingSpec mSpec;
    int64_t mOutputPeriod;
    int64_t mLastPublishTime;
  };

  struct RateInfo {
    drc::LcmWrapper::Ptr mDestLcmWrapper;
    RateInfoSpec mSpec;
    int64_t mOutputPeriod = 0;
    int64_t mLastPublishTime = 0;
    int32_t mMessagesReceived = 0;
    bool mActive = false;
  };

  struct BindingList {
    typedef std::shared_ptr<BindingList> Ptr;

    lcm::Subscription* mSubscription;
    std::vector<Binding> mBindings;
    RateInfo mRateInfo;
    Imp* mImp;

    void handler(const lcm::ReceiveBuffer* iBuffer,
                 const std::string& iChannel) {
      int64_t currentTime = drc::Clock::instance()->getCurrentTime();
      for (auto& binding : mBindings) {
        auto& spec = binding.mSpec;
        if ((binding.mOutputPeriod > 0) &&
            ((currentTime-binding.mLastPublishTime) < binding.mOutputPeriod)) {
          continue;
        }
        binding.mDestLcmWrapper->get()->
          publish(spec.mOutputChannel, iBuffer->data, iBuffer->data_size);
        binding.mLastPublishTime = currentTime;
        if (mImp->mVerbose) {
          std::cout << "transferred (" << spec.mInputCommunity << "," <<
            spec.mInputChannel << ") to (" << spec.mOutputCommunity << "," <<
            spec.mOutputChannel << ")" << std::endl;
        }
      }
      if (mRateInfo.mActive) {
        ++mRateInfo.mMessagesReceived;
        int64_t dt = currentTime - mRateInfo.mLastPublishTime;
        if (dt >= mRateInfo.mOutputPeriod) {
          drc::message_rate_t msg;
          msg.channel = mRateInfo.mSpec.mEnumValue;
          msg.rate_deci_hz = 10.0*mRateInfo.mMessagesReceived/dt*1e6;
          mRateInfo.mDestLcmWrapper->get()->
            publish(mRateInfo.mSpec.mOutputChannel, &msg);
          mRateInfo.mLastPublishTime = currentTime;
          mRateInfo.mMessagesReceived = 0;
        }
      }
    }
  };

  struct Community {
    typedef std::shared_ptr<Community> Ptr;

    std::string mName;
    std::string mUrl;
    bool mIsSource;
    drc::LcmWrapper::Ptr mLcmWrapper;
    std::unordered_map<std::string, BindingList::Ptr> mBindingLists;
    std::unordered_map<std::string, lcm::Subscription*> mSubscriptions;

    bool start() {
      if (mLcmWrapper->isThreadRunning()) return false;
      for (auto iter : mBindingLists) {
        iter.second->mSubscription =
          mLcmWrapper->get()->subscribe(iter.first, &BindingList::handler,
                                        iter.second.get());
      }
      if (mIsSource) return mLcmWrapper->startHandleThread();
      return true;
    }

    bool stop() {
      if (!mLcmWrapper->isThreadRunning()) return false;
      for (auto iter : mSubscriptions) {
        mLcmWrapper->get()->unsubscribe(iter.second);
      }
      if (mIsSource) return mLcmWrapper->stopHandleThread();
      return true;
    }
  };

  std::unordered_map<std::string, std::shared_ptr<Community>> mCommunities;
  bool mVerbose;
};


Bridge::
Bridge() {
  mImp.reset(new Imp());
  setVerbose(false);
}

Bridge::
~Bridge() {
  stop();
}

void Bridge::
setVerbose(const bool iVal) {
  mImp->mVerbose = iVal;
}

bool Bridge::
addCommunity(const std::string& iName, const std::string& iUrl) {
  if (mImp->mCommunities.find(iName) != mImp->mCommunities.end()) {
    std::cout << "error: community " << iName << " already exists on url " <<
      mImp->mCommunities[iName]->mUrl << std::endl;
    return false;
  }

  Imp::Community::Ptr community(new Imp::Community());
  community->mName = iName;
  community->mUrl = iUrl;
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM(iUrl));
  community->mLcmWrapper.reset(new drc::LcmWrapper(lcm));
  mImp->mCommunities[iName] = community;
  return true;
}

bool Bridge::
addBinding(const BindingSpec& iSpec) {
  auto inputIter = mImp->mCommunities.find(iSpec.mInputCommunity);
  if (inputIter == mImp->mCommunities.end()) {
    std::cout << "error: community " << iSpec.mInputCommunity <<
      " does not exist." << std::endl;
    return false;
  }

  auto outputIter = mImp->mCommunities.find(iSpec.mOutputCommunity);
  if (outputIter == mImp->mCommunities.end()) {
    std::cout << "error: community " << iSpec.mOutputCommunity <<
      " does not exist." << std::endl;
    return false;
  }

  auto inputCommunity = inputIter->second;
  auto outputCommunity = outputIter->second;

  Imp::Binding binding;
  binding.mDestLcmWrapper = outputCommunity->mLcmWrapper;
  binding.mSpec = iSpec;
  binding.mSpec.mOutputChannel = iSpec.mOutputChannel.length() > 0 ?
    iSpec.mOutputChannel : iSpec.mInputChannel;
  binding.mOutputPeriod = (iSpec.mOutputFrequency == 0) ? 0 :
    1.0/iSpec.mOutputFrequency*1e6;
  binding.mLastPublishTime = 0;
  
  if (inputCommunity->mBindingLists[iSpec.mInputChannel] == NULL) {
    inputCommunity->mBindingLists[iSpec.mInputChannel].reset
      (new Imp::BindingList());
  }
  inputCommunity->mBindingLists[iSpec.mInputChannel]->mBindings.
    push_back(binding);
  inputCommunity->mBindingLists[iSpec.mInputChannel]->mImp = mImp.get();

  inputCommunity->mIsSource = true;
  
  return true;
}

bool Bridge::
addRateInfo(const RateInfoSpec& iSpec) {
  auto inputIter = mImp->mCommunities.find(iSpec.mInputCommunity);
  if (inputIter == mImp->mCommunities.end()) {
    std::cout << "error: community " << iSpec.mInputCommunity <<
      " does not exist." << std::endl;
    return false;
  }

  auto outputIter = mImp->mCommunities.find(iSpec.mOutputCommunity);
  if (outputIter == mImp->mCommunities.end()) {
    std::cout << "error: community " << iSpec.mOutputCommunity <<
      " does not exist." << std::endl;
    return false;
  }

  auto inputCommunity = inputIter->second;
  auto outputCommunity = outputIter->second;

  auto& rateInfo =
    inputCommunity->mBindingLists[iSpec.mInputChannel]->mRateInfo;
  rateInfo.mSpec = iSpec;
  rateInfo.mDestLcmWrapper = outputCommunity->mLcmWrapper;
  rateInfo.mOutputPeriod = 1.0/iSpec.mOutputFrequency*1e6;
  rateInfo.mActive = true;
  
  return true;
}

bool Bridge::
start() {
  bool success = true;
  for (auto iter : mImp->mCommunities) {
    if (!iter.second->start()) success = false;
  }
  return success;
}

bool Bridge::
stop() {
  bool success = true;
  for (auto iter : mImp->mCommunities) {
    if (!iter.second->stop()) success = false;
  }
  return success;
}
