// 1. find out the map-server process id:
//drc-printf-request | grep map
//extra maps-server	343092855
// 2. then request the map server printf:
// drc--printf-request -r 343092855


#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <string>
#include <sstream>      // std::stringstream
#include <map>
#include <sys/time.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_procman/printf_t.hpp>
#include <lcmtypes/bot_procman/info_t.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <ConciseArgs>

using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, int request_);
    
    ~Pass(){
    }    
        
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void replyHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::printf_reply_t* msg);   
    void infoHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_procman::info_t* msg);   
       
    std::map<int, boost::circular_buffer<string> > pmap_;
    
    bool got_robot;
    bool got_extra;
    int request_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, int request_): 
    lcm_(lcm_), request_(request_){

      got_robot=false;
      got_extra=false;
      
    lcm_->subscribe( "PMD_INFO" ,&Pass::infoHandler,this);  
    lcm_->subscribe( "PMD_PRINTF_REPLY" ,&Pass::replyHandler,this);  
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


void Pass::replyHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::printf_reply_t* msg){

  //* Replicate the messages locally for the sheriff
  for(size_t i=0; i <msg->printfs.size() ; i++){
      std::cout << i<<": " << msg->printfs[i] << "\n";
      bot_procman::printf_t msg_out;
      msg_out.utime = _timestamp_now();
      if (msg->robot){
        msg_out.deputy_name = "robot";
      }else{
        msg_out.deputy_name = "extra"; 
      }
      msg_out.sheriff_id = msg->sheriff_id;
      msg_out.text = msg->printfs[i];
      lcm_->publish("PMD_PRINTF", &msg_out);
  }     
  
  std::string deputy_name;
  if (msg->robot){
    deputy_name = "robot";
  }else{
    deputy_name = "extra"; 
  }
  
  
  std::cout << "Reply for: " << msg->sheriff_id << " | " << deputy_name << "\n";
  if (msg->printfs.size() == 0){
    std::cout << "<--- no messages cached for this process --->\n"; 
  }
  for(size_t i=0; i <msg->printfs.size() ; i++){
      string string_last = msg->printfs[i];
      string_last.erase(std::remove(string_last.begin(), string_last.end(), '\n'), string_last.end());
      std::cout << i<<": " << string_last << "\n";
  }	
  exit(-1);
}

void Pass::infoHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_procman::info_t* msg){

  if (request_ > 0){
    drc::printf_request_t req;
    req.sheriff_id = request_;
    lcm_->publish("PMD_PRINTF_REQUEST",&req);
    std::cout << "Sent the request for "<< request_ << "\n";
    request_=0;
  } else if(request_==0){
    
  } else {

    
    if(msg->host == "robot"){
      if (!got_robot){
	for(size_t i=0; i <msg->cmds.size() ; i++){
	  std::cout << "robot " << msg->cmds[i].nickname << "\t" << msg->cmds[i].sheriff_id << "\n";
	}	
	got_robot =true;
      }
    }
      
    if(msg->host == "extra"){
      if (!got_extra){
	for(size_t i=0; i <msg->cmds.size() ; i++){
	  std::cout << "extra "<< msg->cmds[i].nickname << "\t" << msg->cmds[i].sheriff_id << "\n";
	}	
	got_extra=true;
      }
    }
    
    if(got_extra && got_robot){
      exit(-1); 
    }
    
  }
  
}

int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  int request=-1;
  parser.add(request, "r", "request", "sheriff_id request");
  parser.parse();
  cout << request << " is request\n";

  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
//  exit(-1);
  
  Pass app(lcm, request);
  cout << "printfHandler Ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}


// if of size 3:
// push_back  ... puts in [2] and removes [0]
// pu

//boost circular buffer
// signal tap.cpp
