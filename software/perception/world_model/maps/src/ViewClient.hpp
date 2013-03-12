#ifndef _maps_ViewClient_hpp_
#define _maps_ViewClient_hpp_

#include <string>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>
#include "ViewBase.hpp"

namespace lcm {
  class LCM;
}

namespace maps {

class ViewClient {
public:
  typedef boost::shared_ptr<ViewBase> ViewPtr;

  class Listener {
  public:
    virtual void notifyData(const int64_t iId) = 0;
    virtual void notifyCatalog(const bool iChanged) = 0;
  };

protected:
  typedef std::map<int64_t,ViewPtr> ViewCollection;
  typedef std::map<int64_t,ViewBase::Spec> SpecCollection;

  struct Worker;

public:
  ViewClient();
  virtual ~ViewClient();

  void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);
  void setRequestChannel(const std::string& iChannel);
  void setOctreeChannel(const std::string& iChannel);
  void setCloudChannel(const std::string& iChannel);
  void setRangeChannel(const std::string& iChannel);
  void setCatalogChannel(const std::string& iChannel);

  int64_t request(const ViewBase::Spec& iSpec);

  ViewPtr getView(const int64_t iId) const;
  std::vector<ViewPtr> getAllViews() const;

  bool getSpec(const int64_t iId, ViewBase::Spec& oSpec) const;
  std::vector<ViewBase::Spec> getAllSpecs() const;

  bool addListener(const Listener* iListener);
  bool removeListener(const Listener* iListener);
  bool removeAllListeners();

  bool start();
  bool stop();

protected:
  void notifyCatalogListeners(const bool iChanged);
  void notifyDataListeners(const int64_t iId);


protected:
  std::string mRequestChannel;
  std::string mOctreeChannel;
  std::string mCloudChannel;
  std::string mRangeChannel;
  std::string mCatalogChannel;
  boost::shared_ptr<lcm::LCM> mLcm;

  boost::shared_ptr<Worker> mWorker;
  ViewCollection mViews;
  SpecCollection mCatalog;
  std::set<Listener*> mListeners;
};

}

#endif
