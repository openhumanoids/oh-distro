#ifndef _maps_Collector_hpp_
#define _maps_Collector_hpp_

#include <boost/shared_ptr.hpp>
#include "SensorDataReceiver.hpp"

namespace lcm {
  class LCM;
}

namespace maps {

class SensorDataReceiver;
class MapManager;

class Collector {
public:
  struct DataListener {
    virtual void notify(const SensorDataReceiver::SensorData& iData) = 0;
  };

public:
  Collector();
  ~Collector();

  void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);
  bool start();
  bool stop();

  void addListener(const DataListener& iListener);
  void removeListener(const DataListener& iListener);

  boost::shared_ptr<SensorDataReceiver> getDataReceiver() const;
  boost::shared_ptr<MapManager> getMapManager() const;

  // convenience methods
  bool getLatestFullSweep(int64_t& oStartTime, int64_t& oEndTime) const;
  bool getLatestSwath(const float iMinAngle, const float iMaxAngle,
                      int64_t& oStartTime, int64_t& oEndTime) const;

protected:
  struct Helper;
  boost::shared_ptr<Helper> mHelper;
};

}

#endif
