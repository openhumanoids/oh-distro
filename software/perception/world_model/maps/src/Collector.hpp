#ifndef _maps_Collector_h_
#define _maps_Collector_h_

#include <boost/shared_ptr.hpp>

namespace lcm {
  class LCM;
}

namespace maps {

class SensorDataReceiver;
class MapManager;

class Collector {
public:
  Collector();
  ~Collector();

  void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);
  bool start();
  bool stop();

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
