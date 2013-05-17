#ifndef _LcmWrapper_hpp_
#define _LcmWrapper_hpp_

#include <memory>
#include <boost/shared_ptr.hpp>

namespace lcm {
  class LCM;
}

typedef struct _lcm_t lcm_t;

namespace drc {
  
class LcmWrapper {
public:
  // creates new lcm
  LcmWrapper();

  // uses existing native c object
  LcmWrapper(const lcm_t* iLcm);

  // uses existing shared pointer
  LcmWrapper(const std::shared_ptr<lcm::LCM>& iLcm);

  // uses existing boost pointer
  LcmWrapper(const boost::shared_ptr<lcm::LCM>& iLcm);

  ~LcmWrapper();

  // start/stop messange handling thread
  bool startHandleThread(const bool iJoined=false);
  bool stopHandleThread();
  bool isThreadRunning() const;

  // get shared pointer to c++ object
  std::shared_ptr<lcm::LCM> get() const;

  // get boost shared pointer to c++ object
  boost::shared_ptr<lcm::LCM> getBoost() const;

  // get native c structure
  lcm_t* getC() const;

protected:
  struct Helper;
  std::shared_ptr<Helper> mHelper;
};

}

#endif
