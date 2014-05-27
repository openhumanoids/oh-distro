#include <iostream>
#include <math.h>
#include <fstream>
#include <numeric>

#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/operator
#include "boost/shared_ptr.hpp"
#include <goby/common/time.h>

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc/bandwidth_stats_t.hpp"
#include "lcmtypes/drc/score_t.hpp"

using namespace std;

using namespace boost::posix_time;


class AnalyzeScore{
  public:
    AnalyzeScore(boost::shared_ptr<lcm::LCM> &lcm_, const std::string& output_prefix);
    
    ~AnalyzeScore(){
    }    
        
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    
    void statsHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const drc::bandwidth_stats_t* msg);


    std::ofstream sent_out_;
    std::ofstream queued_out_;

};

AnalyzeScore::AnalyzeScore(boost::shared_ptr<lcm::LCM> &lcm_, const std::string& output_prefix): 
    lcm_(lcm_),
    sent_out_(std::string(output_prefix + "_sent_bytes.csv").c_str()),
    queued_out_(std::string(output_prefix + "_queued_bytes.csv").c_str())
{  
  lcm_->subscribe("BASE_BW_STATS", &AnalyzeScore::statsHandler,this);  
  lcm_->subscribe("ROBOT_BW_STATS", &AnalyzeScore::statsHandler,this);  
}

void AnalyzeScore::statsHandler(const lcm::ReceiveBuffer* rbuf, 
                                const std::string& channel,
                                const drc::bandwidth_stats_t* msg)
{
    static int bw_i = 0;

    if(bw_i == 0)
    {
        sent_out_ << "UTIME" << " ";
        queued_out_ << "UTIME" << " ";
        for(int i = 0, n = msg->num_sent_channels; i < n; ++i)
        {
            sent_out_ << msg->sent_channels[i] << " ";
            queued_out_ << msg->sent_channels[i] << " ";
        }
        sent_out_ << std::endl;
        queued_out_ << std::endl;
    }

    sent_out_ << msg->utime;
    for(int i = 0, n = msg->num_sent_channels; i < n; ++i)
        sent_out_  << "," << msg->sent_bytes[i];
    sent_out_ << std::endl;

    queued_out_ << msg->utime;
    for(int i = 0, n = msg->num_sent_channels; i < n; ++i)
        queued_out_ << "," << msg->queued_bytes[i] ;
    queued_out_ << std::endl;
    
    ++bw_i;
}


int main(int argc, char ** argv) {
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM()  );
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  if(argc < 2)
  {
      std::cerr <<"Usage: drc2013-analyze-network-score output_prefix" <<std::endl;
      exit(1);
  }
  
      
  AnalyzeScore app(lcm, argv[1]);

  while(0 == lcm->handle());
  
  return 0; 
}
