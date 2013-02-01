#ifndef _maps_ViewClient_hpp_
#define _maps_ViewClient_hpp_

#include <string>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>

namespace lcm {
  class LCM;
}

namespace maps {

class MapView;

class ViewClient {
public:
  typedef boost::shared_ptr<MapView> MapViewPtr;
  typedef boost::shared_ptr<lcm::LCM> LcmPtr;

  class Listener {
  public:
    virtual void notify(const int64_t iId) = 0;
  };

protected:
  typedef std::map<int64_t,MapViewPtr> MapViewCollection;

  struct Worker;

public:
  ViewClient();
  virtual ~ViewClient();

  void setLcm(const LcmPtr& iLcm);
  void setOctreeChannel(const std::string& iChannel);
  void setCloudChannel(const std::string& iChannel);

  MapViewPtr getView(const int64_t iId) const;
  std::vector<MapViewPtr> getAllViews() const;

  bool addListener(const Listener* iListener);
  bool removeListener(const Listener* iListener);
  bool removeAllListeners();

  bool start();
  bool stop();


protected:
  std::string mOctreeChannel;
  std::string mCloudChannel;

  boost::shared_ptr<Worker> mWorker;
  MapViewCollection mViews;
  std::set<Listener*> mListeners;
};

}

#endif
