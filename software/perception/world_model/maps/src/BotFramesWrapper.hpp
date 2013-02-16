#ifndef _maps_BotFramesWrapper_hpp_
#define _maps_BotFramesWrapper_hpp_

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

// forward declarations
namespace lcm {
  class LCM;
}

namespace maps {

class BotFramesWrapper {
protected:
  struct BotStructures;

public:
  BotFramesWrapper();
  ~BotFramesWrapper();

  void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);

  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    const int64_t iTimestamp, Eigen::Isometry3d& oTransform);
  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    const int64_t iTimestamp, Eigen::Isometry3f& oTransform);
  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    const int64_t iTimestamp,
                    Eigen::Vector3d& oTrans, Eigen::Quaterniond& oRot);
  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    const int64_t iTimestamp,
                    Eigen::Vector3f& oTrans, Eigen::Quaternionf& oRot);

protected:
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<BotStructures> mBotStructures;
};

}

#endif
