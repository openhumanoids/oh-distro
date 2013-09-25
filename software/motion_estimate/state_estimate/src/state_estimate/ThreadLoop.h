#ifndef __ThreadLoop_h
#define __ThreadLoop_h

#include "SharedPtrMacros.h"
#include <boost/thread/thread.hpp>
#include <iostream>

namespace StateEstimate
{

//----------------------------------------------------------------------------
class ThreadLoop
{
public:

  ThreadLoop()
  {

  }

  void start()
  {
    if (this->Thread)
    {
      std::cout << "Error: Start() called while thread is active." << std::endl;
      return;
    }

    this->ShouldStop = false;
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&ThreadLoop::run, this)));
  }

  void stop()
  {
    if (this->Thread)
      {
      this->ShouldStop = true;
      this->Thread->join();
      this->Thread.reset();
      }
  }

protected:

  virtual void run() = 0;

  bool ShouldStop;
  boost::shared_ptr<boost::thread> Thread;
};

} // end namespace

#endif // __ThreadLoop_h
