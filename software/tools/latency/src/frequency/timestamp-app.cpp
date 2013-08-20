#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/multisense.hpp"

#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm, int mode_, bool verbose);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const  drc::atlas_state_t * msg);
  void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const  multisense::state_t * msg);
  
  int mode_;
  bool verbose_;
  
  std::vector <int64_t> utimes;
  
private:
  
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, int mode_, bool verbose): _lcm(_lcm),
  mode_(mode_), verbose_(verbose_){
  
  _lcm->subscribe("ATLAS_STATE", &App::atlasHandler, this);
  _lcm->subscribe("MULTISENSE_STATE", &App::multisenseHandler, this);
  
  utimes.assign(2,0);
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

std::string blank = "               ";
void App::atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  utimes[0] = msg->utime;
  
  int64_t utime_now = _timestamp_now();
  std::cout << utime_now << "," << utimes[0]-utime_now << "," << 0 << "\n";
}

void App::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg){
  utimes[1] = msg->utime;

  int64_t utime_now = _timestamp_now();
  std::cout << utime_now << "," << 0 << "," << utimes[1]-utime_now << "\n";

}

int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  int mode=0;
  bool verbose=false;
  parser.add(mode, "m", "mode", "Mode [0,1]");
  parser.add(verbose, "v", "verbose", "Verbose");
  parser.parse();
  cout << "mode is: " << mode << "\n"; 
  cout << "verbose is: " << verbose << "\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  App app(lcm, mode, verbose);
  cout << "App ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
