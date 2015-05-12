#ifndef __Bridge_hpp__
#define __Bridge_hpp__

#include <string>
#include <memory>

class Bridge {
public:
  struct BindingSpec {
    std::string mInputCommunity;
    std::string mInputChannel;
    std::string mOutputCommunity;
    std::string mOutputChannel;
    float mOutputFrequency;
  };

  struct RateInfoSpec {
    std::string mInputCommunity;
    std::string mInputChannel;
    std::string mOutputCommunity;
    std::string mOutputChannel;
    float mOutputFrequency;
    int mEnumValue;  // from drc_message_rate_t enum
  };

public:
  Bridge();
  ~Bridge();

  void setVerbose(const bool iVal);
  bool addCommunity(const std::string& iName, const std::string& iUrl);
  bool addBinding(const BindingSpec& iSpec);
  bool addRateInfo(const RateInfoSpec& iSpec);

  bool start();
  bool stop();

protected:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

#endif
