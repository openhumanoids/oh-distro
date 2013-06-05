#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
//#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/drc/utime_t.hpp"
#include "lcmtypes/drc/frequency_t.hpp"
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
  
  int64_t ratecheck_wall_utime_, ratecheck_sim_utime_;
  
private:
  Frequency* frequency_;  
  
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, int mode_, bool verbose): _lcm(_lcm),mode_(mode_){
  
  frequency_ = new Frequency(0.5, verbose, "LCM");  
  frequency_->readChannels();

  _lcm->subscribe("ROBOT_UTIME", &App::handleUtime, this); 
  _lcm->subscribe(".*", &App::handleAllMsg, this); 
  
  
  // 
  ratecheck_wall_utime_ = 0;
  ratecheck_sim_utime_ = 0;
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
    
//    out.channel = chans;
    
    for (size_t i=0; i < chans.size() ;i++){
      if (chans[i] == "EST_ROBOT_STATE"){ out.channel.push_back( (int8_t)  drc::frequency_t::EST_ROBOT_STATE) ;
      }else if (chans[i] == "ATLAS_COMMAND"){    out.channel.push_back( (int8_t) drc::frequency_t::ATLAS_COMMAND) ;
      }else if (chans[i] == "CAMERA"){ out.channel.push_back( (int8_t) drc::frequency_t::CAMERA) ;
      }else if (chans[i] == "CAMERA_LHAND"){ out.channel.push_back( (int8_t) drc::frequency_t::CAMERA_LHAND) ;
      }else if (chans[i] == "CAMERA_RHAND"){ out.channel.push_back( (int8_t) drc::frequency_t::CAMERA_RHAND) ;
      }else if (chans[i] == "SCAN"){ out.channel.push_back( (int8_t) drc::frequency_t::SCAN) ;
      }else{ out.channel.push_back( (int8_t) drc::frequency_t::UNKNOWN) ; 
      }
    }
    
    
    int64_t curr_wall_time = _timestamp_now();
    float frac = float( msg->utime - ratecheck_sim_utime_ ) / float(curr_wall_time  - ratecheck_wall_utime_);
    std::cout << frac << " real time rate\n";
    ratecheck_wall_utime_ = curr_wall_time;
    ratecheck_sim_utime_ = msg->utime;
    out.real_time_percent= (int8_t) 100.0*frac; // will be a percetn 0-100
    
    _lcm->publish("FREQUENCY_LCM", &out);
  }
  
  
 

/*
 *  if (m.utime > last_10hz_rbot_time + 100000):
    #print "================="
    #print m.utime
    #print last_10hz_rbot_time
    #print float(m.utime - last_10hz_rbot_time)
    #print float(curr_wall_time  - last_10hz_rbot_time)
    frac = float( m.utime - last_10hz_rbot_time ) / float(curr_wall_time  - last_10hz_wall_time)
    #print frac
    utime_10hz_rate.append(m.utime,frac)
    last_10hz_wall_time = curr_wall_time
    last_10hz_rbot_time = m.utime  */
 
 
  
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