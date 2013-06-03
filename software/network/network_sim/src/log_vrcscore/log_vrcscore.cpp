#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <fstream>


#include <boost/shared_ptr.hpp>
#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/operator
#include <lcm/lcm-cpp.hpp>

#include <bot_core/bot_core.h>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pointcloud_tools/pointcloud_math.hpp>

#include <ConciseArgs>

using namespace std;

using namespace boost::posix_time;

char* pHome = getenv("HOME");  
string home = string(pHome);


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
        
  private:
    int verbose_;
    int64_t last_utime_;
    boost::shared_ptr<lcm::LCM> lcm_;
    void vrcScoreHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::score_t* msg);   
    void resetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg);   
    
    std::ofstream data_usage_log_;
    std::stringstream header_string_;
    
    void open_usage_log();
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): 
    lcm_(lcm_){
  
  lcm_->subscribe( "VRC_SCORE" ,&Pass::vrcScoreHandler,this);
  lcm_->subscribe( "RESET_SHAPER_STATS" ,&Pass::resetHandler,this);  
  
  last_utime_ =0;
  
  open_usage_log();
}

void Pass::open_usage_log(){
  string pathname = string(home + "/drc/software/config/");

  std::string data_usage_file_name = pathname + "vrc-score-" + to_iso_string(second_clock::universal_time()) + ".csv";
  data_usage_log_.open(data_usage_file_name.c_str()  );
  
  if(!data_usage_log_.is_open())
  {
    std::cout << "Failed to open requested CSV data log file: " << data_usage_file_name << ". Check value and permissions on --logpath" << std::endl;
    exit(-1);
  }else{
    std::cout << "Writing " << data_usage_file_name << "\n";
  }
  
  if(data_usage_log_.is_open()){
    header_string_ << "UTIME,BYTESDOWN,BYTESUP" << std::endl;
    data_usage_log_ << header_string_.str();
    data_usage_log_.flush();
  }
}


void Pass::resetHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::utime_t* msg){
  data_usage_log_.close();
  std::cout << "RESET_SHAPER_STATS received, starting new file\n";
  open_usage_log();
}



int counter =0;
void Pass::vrcScoreHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::score_t* msg){
  
  if (counter % 10 ==0){
    std::cout << "VRC_SCORE "<< counter<< "received\n";
  }
  counter++;
  
  if(data_usage_log_.is_open()){
    data_usage_log_ << msg->utime << "," 
                    << msg->bytes_downlink_remaining  << ","
		    << msg->bytes_uplink_remaining  <<"\n";
    data_usage_log_.flush();
  }else{ 
    std::cout << "not longer writing\n";
  }
}


int main(int argc, char ** argv) {
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM()  );
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Ready to write VRC_SCORE plan" << endl << "============================" << endl;
  while(0 == lcm->handle());
  
  
  
  return 0; 
}
