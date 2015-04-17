#ifndef _drc_BotWrapper_hpp_
#define _drc_BotWrapper_hpp_

#include <memory>
#include <vector>
#include <Eigen/Geometry>

namespace lcm {
  class LCM;
  class Subscription;
}

typedef struct _BotParam BotParam;
typedef struct _BotFrames BotFrames;
typedef struct _lcm_t lcm_t;

namespace drc {

class BotWrapper {
public:
  typedef std::shared_ptr<BotWrapper> Ptr;

public:
  BotWrapper();
  BotWrapper(const std::shared_ptr<lcm::LCM>& iLcm,
             const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);
  BotWrapper(const lcm_t* iLcm,
             const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);

  void setDefaults();
  void set(const std::shared_ptr<lcm::LCM>& iLcm,
           const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);
  void set(const lcm_t* iLcm,
           const BotParam* iBotParam=NULL, const BotFrames* iBotFrames=NULL);

  std::shared_ptr<lcm::LCM> getLcm() const;
  BotParam* getBotParam() const;
  BotFrames* getBotFrames() const;

  int64_t getCurrentTime() const;

  // for bot frames
  int64_t getLatestTime(const std::string& iFrom, const std::string& iTo) const;
  template <typename T>
  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    Eigen::Transform<T,3,Eigen::Isometry>& oTransform,
                    const int64_t iTime=-1) const;
  template <typename T>
  bool getTransform(const std::string& iFrom, const std::string& iTo,
                    Eigen::Quaternion<T>& oRot, Eigen::Matrix<T,3,1>& oTrans,
                    const int64_t iTime=-1) const;

  // for bot param
  bool hasKey(const std::string& iKey) const;
  bool set(const std::string& iKey, const std::string& iValue);
  bool set(const std::string& iKey, const int iValue);
  bool set(const std::string& iKey, const double iValue);
  bool set(const std::string& iKey, const bool iValue);
  std::string get(const std::string& iKey) const;
  int getInt(const std::string& iKey) const;
  double getDouble(const std::string& iKey) const;
  bool getBool(const std::string& iKey) const;
  std::vector<double> getDoubles(const std::string& iKey) const;
  std::vector<std::string> getStrings(const std::string& iKey) const;
  bool get(const std::string& iKey, std::string& oValue) const;
  bool get(const std::string& iKey, int& oValue) const;
  bool get(const std::string& iKey, double& oValue) const;
  bool get(const std::string& iKey, bool& oValue) const;
  bool get(const std::string& iKey, std::vector<double>& oValues) const;
  bool get(const std::string& iKey, std::vector<std::string>& oValues) const;
  std::vector<std::string> getKeys(const std::string& iKey) const;

protected:
  std::shared_ptr<lcm::LCM> mLcm;
  BotParam* mBotParam;
  BotFrames* mBotFrames;
  lcm::Subscription* mTimeSubscription;
};

}
#endif
