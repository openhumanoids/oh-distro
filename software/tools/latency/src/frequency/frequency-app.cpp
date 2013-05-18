#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"
#include <frequency/frequency.hpp>


#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm, int mode_, bool verbose);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void handleUtime(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const  drc::utime_t * msg);
  void handleAllMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan);
  
  int mode_;
  std::string message_;
  
private:
  Frequency* frequency_;  
  
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, int mode_, bool verbose): _lcm(_lcm),mode_(mode_){
  
  frequency_ = new Frequency(0.5, verbose, "LCM");  
  frequency_->readChannels();

  _lcm->subscribe("ROBOT_UTIME", &App::handleUtime, this); 
  _lcm->subscribe(".*", &App::handleAllMsg, this); 
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void App::handleUtime(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const  drc::utime_t * msg)  {
  std::vector < float > freqs;
  if (frequency_->setUtime(msg->utime , freqs ) ) {
    drc::frequency_t out;
    out.utime = msg->utime;
    for (size_t i=0;i < freqs.size() ;i++){
      out.frequency.push_back( (int16_t) round( freqs[i]) );
    }
    out.num = out.frequency.size();
    std::vector< std::string> chans = frequency_->getChannels();
    out.channel = chans;
    _lcm->publish("FREQUENCY_LCM", &out);
  }
}

void App::handleAllMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan){
  frequency_->incrementCounter( chan );
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