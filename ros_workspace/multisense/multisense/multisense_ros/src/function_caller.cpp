#include <multisense_ros/function_caller.h>

namespace multisense_ros
{

FunctionCaller::FunctionCaller(unsigned int thread_pool_size, unsigned int max_queue_size)
  :max_queue_size_(max_queue_size), running(true)
{
  threads.resize(thread_pool_size);
  for (unsigned i=0; i<threads.size(); i++)
    threads[i] = new boost::thread(boost::bind(&FunctionCaller::threadLoop, this, i));
}


FunctionCaller::~FunctionCaller()
{
  // notify all threads that are locked on condition variable
  {
    boost::mutex::scoped_lock lock(mutex);
    running = false;
    condition.notify_all();
  }

  for (unsigned i=0; i<threads.size(); i++)
  {
    threads[i]->join();
    delete threads[i];
  }
}

void FunctionCaller::addFunction(boost::function<void(void)> func)
{
  boost::mutex::scoped_lock lock(mutex);

  assert(queue.size() <= max_queue_size_);

  while (queue.size() >= max_queue_size_)
    queue.pop_back();

  queue.push_front(func);
  //printf("FunctionCaller ADD    ==> %5zu\n", queue.size());
  condition.notify_one();
}


void FunctionCaller::threadLoop(unsigned i)
{
  while (true)
  {
    boost::function<void(void)> f;
    {
      // wait for condition if queue is empty
      boost::mutex::scoped_lock lock(mutex);
      while (queue.empty())
      {
        if (!running)
          return;
        condition.wait(lock);
      }
      if (!running)
        return;

      assert(!queue.empty());
      f = queue.back();
      queue.pop_back();
      //printf("FunctionCaller REMOVE ==> %5zu\n", queue.size());
    }
    // call actual function
    f();
  }
}

}
