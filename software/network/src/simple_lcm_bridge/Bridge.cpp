#include "Bridge.hpp"

#include <iostream>
#include <unordered_map>
#include <vector>
#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>

struct Bridge::Imp {
  struct Binding {
    drc::LcmWrapper::Ptr mDestLcmWrapper;
    std::string mDestChannel;
  };

  struct BindingList {
    lcm::Subscription* mSubscription;
    std::vector<Binding> mBindings;

    void handler(const lcm::ReceiveBuffer* iBuffer,
                 const std::string& iChannel) {
      for (auto binding : mBindings) {
        binding.mDestLcmWrapper->get()->
          publish(binding.mDestChannel, iBuffer->data, iBuffer->data_size);
      }
    }
  };

  struct Community {
    typedef std::shared_ptr<Community> Ptr;

    std::string mName;
    std::string mUrl;
    bool mIsSource;
    drc::LcmWrapper::Ptr mLcmWrapper;
    std::unordered_map<std::string, BindingList> mBindingLists;
    std::unordered_map<std::string, lcm::Subscription*> mSubscriptions;

    bool start() {
      if (mLcmWrapper->isThreadRunning()) return false;
      for (auto iter : mBindingLists) {
        iter.second.mSubscription =
          mLcmWrapper->get()->subscribe(iter.first, &BindingList::handler,
                                        &iter.second);
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
};


Bridge::
Bridge() {
  mImp.reset(new Imp());
}

Bridge::
~Bridge() {
  stop();
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
  binding.mDestChannel = iSpec.mOutputChannel.length() > 0 ?
    iSpec.mOutputChannel : iSpec.mInputChannel;
  
  inputCommunity->mBindingLists[iSpec.mInputChannel].mBindings.
    push_back(binding);

  inputCommunity->mIsSource = true;
  
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
