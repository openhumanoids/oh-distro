#ifndef __LCMThread_h
#define __LCMThread_h

#include "ThreadLoop.h"

#include <lcm/lcm-cpp.hpp>
#include <sys/select.h>

namespace StateEstimate
{

//----------------------------------------------------------------------------
class LCMThread : public ThreadLoop
{
public:

  LCMThread()
  {
    this->UseSelect = true;
    this->SelectTimeoutInSeconds = 0.1;
    this->initLCM();
  }

  virtual void run()
  {
    if (this->UseSelect)
    {
      this->handleLoopWithSelect();
    }
    else
    {
      this->handleLoop();
    }
  }

  boost::shared_ptr<lcm::LCM> lcmHandle()
  {
    return this->LCMHandle;
  }

protected:

  bool waitForLCM(double timeout)
  {
    int lcmFd = this->LCMHandle->getFileno();

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout * 1e6;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcmFd, &fds);

    int status = select(lcmFd + 1, &fds, 0, 0, &tv);
    return (status != 0 && FD_ISSET(lcmFd, &fds));
  }

  void handleLoopWithSelect()
  {
    while (!this->ShouldStop)
    {
      bool lcmReady = this->waitForLCM(this->SelectTimeoutInSeconds);

      if (this->ShouldStop)
      {
        break;
      }

      if (lcmReady)
      {
        if (this->LCMHandle->handle() != 0)
        {
          std::cout << "Error: lcm->handle() returned non-zero.  Stopping thread." << std::endl;
          break;
        }
      }
    }
  }

  void handleLoop()
  {
    while (!this->ShouldStop)
    {
      if (this->LCMHandle->handle() != 0)
      {
        std::cout << "Error: lcm->handle() returned non-zero.  Stopping thread." << std::endl;
        break;
      }
    }
  }

  void initLCM()
  {
    if (this->LCMHandle)
    {
      return;
    }

    this->LCMHandle = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
    if (!this->LCMHandle->good())
    {
      std::cout << "Error: lcm is not good()." << std::endl;
    }
  }

  bool UseSelect;
  double SelectTimeoutInSeconds;

  boost::shared_ptr<lcm::LCM> LCMHandle;
};

} // end namespace

#endif // __LCMThread_h
