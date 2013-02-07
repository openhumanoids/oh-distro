#ifndef _maps_ViewClient_hpp_
#define _maps_ViewClient_hpp_

#include <string>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>

#include "MapView.hpp"

namespace lcm {
  class LCM;
}

namespace maps {

class ViewClient {
public:
  typedef boost::shared_ptr<MapView> MapViewPtr;
  typedef boost::shared_ptr<lcm::LCM> LcmPtr;

  class Listener {
  public:
    virtual void notifyData(const int64_t iId) = 0;
    virtual void notifyCatalog(const bool iChanged) = 0;
  };

protected:
  typedef std::map<int64_t,MapViewPtr> MapViewCollection;

  struct Worker;

public:
  ViewClient();
  virtual ~ViewClient();

  void setLcm(const LcmPtr& iLcm);
  void setRequestChannel(const std::string& iChannel);
  void setOctreeChannel(const std::string& iChannel);
  void setCloudChannel(const std::string& iChannel);
  void setCatalogChannel(const std::string& iChannel);

  int64_t request(const MapView::Spec& iSpec);

  MapViewPtr getView(const int64_t iId) const;
  std::vector<MapViewPtr> getAllViews() const;

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
  std::string mCatalogChannel;
  LcmPtr mLcm;

  boost::shared_ptr<Worker> mWorker;
  MapViewCollection mViews;
  std::set<Listener*> mListeners;
};

}

#endif
