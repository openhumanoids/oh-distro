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
    AnalyzeScore(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~AnalyzeScore(){
    }    
        
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    void vrcScoreHandler(const lcm::ReceiveBuffer* rbuf, 
                         const std::string& channel, const  drc::score_t* msg);
    
    void statsHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const drc::bandwidth_stats_t* msg);

    unsigned used_uplink( const drc::bandwidth_stats_t& msg)
        { return std::accumulate(msg.sent_bytes.begin(), msg.sent_bytes.end(), 0);}
    
    unsigned used_downlink( const drc::bandwidth_stats_t& msg)
        { return std::accumulate(msg.received_bytes.begin(), msg.received_bytes.end(), 0);}

    drc::bandwidth_stats_t previous_bw_stats_;
    drc::bandwidth_stats_t out_of_gate_bw_stats_;
    drc::bandwidth_stats_t last_completion_bw_stats_;
    drc::score_t initial_score_;

    bool have_initial_score_;
    bool out_of_gate_;
    int current_goals_completed_;
};

AnalyzeScore::AnalyzeScore(boost::shared_ptr<lcm::LCM> &lcm_): 
    lcm_(lcm_),
    have_initial_score_(false),
    out_of_gate_(false),
    current_goals_completed_(0)
{  
  lcm_->subscribe("BASE_BW_STATS", &AnalyzeScore::statsHandler,this);  
  lcm_->subscribe("VRC_SCORE" ,&AnalyzeScore::vrcScoreHandler,this);
}

void AnalyzeScore::vrcScoreHandler(const lcm::ReceiveBuffer* rbuf, 
                                   const std::string& channel,
                                   const  drc::score_t* msg)
{
    static int score_i = 0;
    if(!have_initial_score_ && msg->bytes_uplink_remaining != 0)
    {
        std::cout << goby::util::as<boost::posix_time::ptime>(static_cast<goby::uint64>(msg->utime))
                  << " Initial bytes (up/down):\t\t\t\t" << msg->bytes_uplink_remaining << "/" <<  msg->bytes_downlink_remaining << std::endl;
        initial_score_ = *msg;
        have_initial_score_ = true;
    }

    if(have_initial_score_ && !out_of_gate_ && (msg->bytes_uplink_remaining < initial_score_.bytes_uplink_remaining || msg->bytes_downlink_remaining < initial_score_.bytes_downlink_remaining))
    {
        out_of_gate_ = true;
        out_of_gate_bw_stats_ = previous_bw_stats_;
        std::cout << goby::util::as<boost::posix_time::ptime>(static_cast<goby::uint64>(msg->utime)) << " Left gate" << std::endl;
    }
        
    
    if(msg->completion_score > current_goals_completed_)
    {
        last_completion_bw_stats_ = previous_bw_stats_;
        current_goals_completed_ = msg->completion_score;
        
        std::cout << goby::util::as<boost::posix_time::ptime>(static_cast<goby::uint64>(msg->utime))
                  << " Score " << msg->completion_score
                  << "; our count of bytes remaining (up/down):\t"
                  << initial_score_.bytes_uplink_remaining-(used_uplink(last_completion_bw_stats_) - used_uplink(out_of_gate_bw_stats_))
                  << "/"
                  << initial_score_.bytes_downlink_remaining-(used_downlink(last_completion_bw_stats_) - used_downlink(out_of_gate_bw_stats_))
                  << std::endl;
        std::cout << goby::util::as<boost::posix_time::ptime>(static_cast<goby::uint64>(msg->utime))
                  << " Score " << msg->completion_score
                  << "; DARPA count of bytes remaining (up/down):\t"
                  << msg->bytes_uplink_remaining
                  << "/"
                  << msg->bytes_downlink_remaining
                  << std::endl;
    }
    
    ++score_i;
}


void AnalyzeScore::statsHandler(const lcm::ReceiveBuffer* rbuf, 
                                const std::string& channel,
                                const drc::bandwidth_stats_t* msg)
{
    static int bw_i = 0;

    previous_bw_stats_ = *msg;
    
    ++bw_i;
}


int main(int argc, char ** argv) {
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM()  );
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  AnalyzeScore app(lcm);

  while(0 == lcm->handle());
  
  return 0; 
}
