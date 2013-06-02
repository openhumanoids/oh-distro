#include <iostream>
#include <boost/thread.hpp>
 
using namespace std;
using boost::thread;
using boost::mutex;
using boost::shared_lock;
using boost::shared_mutex;
using boost::upgrade_lock;
using boost::upgrade_to_unique_lock;

#include <ConciseArgs>

mutex a;
shared_mutex b;
 
const int THREAD_COUNT = 10;
 
void worker(int j) {
  std::cout << "starting " << j << "\n";
  int i=0;
    while (1==1)
      i++;
}
 
int main(int argc, char** argv)
{
  int nthreads=0; // id of this tracker instance. Plane Tracker==0, otherwise above that
  ConciseArgs opt(argc, (char**)argv);
  opt.add(nthreads, "t", "threads","Number of threads");
  opt.parse();
  
  
    thread *threads[nthreads];
 
    upgrade_lock<shared_mutex> lock(b);
    upgrade_to_unique_lock<shared_mutex> uniqueLock(lock);
 
    // Creation
    for(int i = 0; i < nthreads; i++) {
        threads[i] = new thread(worker, i);
    }
 
    cin.get();
    cout << "Unlocking..." << endl;
    uniqueLock.~upgrade_to_unique_lock();
 
    // Cleanup
    for(int i = 0; i < THREAD_COUNT; i++) {
        threads[i]->join();
        delete threads[i];
    }
 
    return 0;
}