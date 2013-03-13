#ifndef _maps_BotWrapper_hpp_
#define _maps_BotWrapper_hpp_

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

namespace lcm {
  class LCM;
}

typedef struct _BotParam BotParam;
typedef struct _BotFrames BotFrames;
typedef struct _lcm_t lcm_t;

namespace maps {

class BotWrapper {
public:
  typedef boost::shared_ptr<BotWrapper> Ptr;

public:
  BotWrapper();
  BotWrapper(const boost::shared_ptr<lcm::LCM>& iLcm,
             const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);
  BotWrapper(const lcm_t* iLcm,
             const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);

  void setDefaults();
  void set(const boost::shared_ptr<lcm::LCM>& iLcm,
           const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);
  void set(const lcm_t* iLcm,
           const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);

  boost::shared_ptr<lcm::LCM> getLcm() const;
  BotParam* getBotParam() const;
  BotFrames* getBotFrames() const;

  int64_t getLatestTime(const std::string& iFrom, const std::string& iTo) const;
  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    Eigen::Isometry3f& oTransform,
                    const int64_t iTime=-1) const;
  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    Eigen::Quaternionf& oRot, Eigen::Vector3f& oTrans,
                    const int64_t iTime=-1) const;


protected:
  boost::shared_ptr<lcm::LCM> mLcm;
  BotParam* mBotParam;
  BotFrames* mBotFrames;
};

}
#endif
