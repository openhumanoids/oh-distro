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
  };

public:
  Bridge();
  ~Bridge();

  bool addCommunity(const std::string& iName, const std::string& iUrl);
  bool addBinding(const BindingSpec& iSpec);

  bool start();
  bool stop();

protected:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

#endif
