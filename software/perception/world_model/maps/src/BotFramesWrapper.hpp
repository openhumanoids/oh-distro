#ifndef _maps_BotFramesWrapper_hpp_
#define _maps_BotFramesWrapper_hpp_

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

// forward declarations
namespace lcm {
  class LCM;
}

typedef struct _BotParam BotParam;
typedef struct _BotFrames BotFrames;

namespace maps {

class BotFramesWrapper {
public:
  BotFramesWrapper(const BotParam* iParam=NULL);
  ~BotFramesWrapper();

  void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);
  void setBotParam(const BotParam* iParam);

  BotFrames* getNative() const;

  int64_t getLatestTimestamp(const std::string& iFrom,
                             const std::string& iTo) const;

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
  BotParam* mBotParam;
  BotFrames* mBotFrames;
};

}

#endif
