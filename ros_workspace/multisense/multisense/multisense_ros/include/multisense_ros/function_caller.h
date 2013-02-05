#ifndef MULTISENSE_ROS_FUNCTION_CALLER_H
#define MULTISENSE_ROS_FUNCTION_CALLER_H

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <deque>
#include <vector>

namespace multisense_ros
{

  class FunctionCaller
  {
  public:
    FunctionCaller(unsigned int thread_pool_size=1, unsigned int max_queue_size=1 );

    ~FunctionCaller();

    void addFunction(boost::function<void(void)> func);

    void threadLoop(unsigned i);

  private:
    boost::condition condition;

    const unsigned int max_queue_size_;
    bool running;
    std::deque<boost::function<void(void)> > queue;
    boost::mutex mutex;
    std::vector<boost::thread*> threads;
  };

}
#endif
