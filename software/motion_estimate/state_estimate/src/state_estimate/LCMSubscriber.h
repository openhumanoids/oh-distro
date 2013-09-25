#ifndef __LCMSubscriber_h
#define __LCMSubscriber_h

#include "Macros.h"
#include "SharedPtrMacros.h"

#include <lcm/lcm-cpp.hpp>

namespace StateEstimate
{

//----------------------------------------------------------------------------
class LCMSubscriber
{
public:

  virtual ~LCMSubscriber()
  {

  }

  virtual void subscribe(boost::shared_ptr<lcm::LCM> lcmHandle)
  {
    VarNotUsed(lcmHandle);
  }

};

} // end namespace

#endif // __LCMSubscriber_h
