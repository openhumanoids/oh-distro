#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>      // std::stringstream
#include <algorithm>


#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <lcm/lcm-cpp.hpp>
#include <map>

#include <lcmtypes/bot_procman/printf_t.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>


using namespace std;


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
        
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void printfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_procman::printf_t* msg);   
    
    void requestHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::printf_request_t* msg);   
    
    void outputBuffer(boost::circular_buffer<string> &cb);
    
    std::map<int, boost::circular_buffer<string> > pmap_;
};


Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): 
    lcm_(lcm_){
  
  lcm_->subscribe( "PMD_PRINTF" ,&Pass::printfHandler,this);
  lcm_->subscribe( "PMD_PRINTF_REQUEST" ,&Pass::requestHandler,this);  
}


void Pass::outputBuffer(boost::circular_buffer<string> &cb){
  for (size_t i=0; i < cb.size() ; i++){
    std::cout << i << ": " << cb[i] << "\n";
  }
  std::cout << "\n";    
}

void Pass::requestHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::printf_request_t* msg){
  if( pmap_.find(msg->sheriff_id) !=pmap_.end() ){ // if it exists
    std::cout << "about to reply to request for "<< msg->sheriff_id << "\n";
    std::map<int, boost::circular_buffer<string> >::iterator it;
    it = pmap_.find(msg->sheriff_id);
    
    drc::printf_reply_t reply;
    for (size_t i=0; i < it->second.size() ; i++){
      std::cout << i << ": " << it->second[i] << "\n";
      reply.printfs.push_back( it->second[i] );
      if (it->second[i].substr(0,1) == "1"){
         reply.robot=true;
      }else{
         reply.robot=false;
      }
    }
    reply.n_printfs = reply.printfs.size();
    reply.sheriff_id = msg->sheriff_id;
    lcm_->publish("PMD_PRINTF_REPLY", &reply);
  }else{
    std::cout << "Requested "<< msg->sheriff_id << ". no cache found\n";
    drc::printf_reply_t reply;
    reply.n_printfs = 0;
    reply.sheriff_id = msg->sheriff_id;
    reply.robot=false; //... actually can't provide this information
    lcm_->publish("PMD_PRINTF_REPLY", &reply);
  }
}


void Pass::printfHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_procman::printf_t* msg){
  std::cout << pmap_.size() << " buffers total\n";
  typedef std::map<int, boost::circular_buffer<string> >::iterator it_type;
  for(it_type iterator = pmap_.begin(); iterator != pmap_.end(); iterator++) {
    std::cout << iterator->first << ", ";
  }  
  std::cout << "\n";
  
  
  // Search for the id, if you dont find it add a new map member:
  if( pmap_.find(msg->sheriff_id) ==pmap_.end() ){
    std::cout << "didnt find "<< msg->sheriff_id <<"\n";
    boost::circular_buffer<string> cb(5);
    pmap_[msg->sheriff_id] = cb;      
  }

  std::map<int, boost::circular_buffer<string> >::iterator it;
  it = pmap_.find(msg->sheriff_id);

  int64_t secs= round(msg->utime/1E6) ;
  int64_t secs_100 =  secs -  100*round(secs/100); // seconds 0-100


  std::string text_str = msg->text;
  
  text_str.erase( std::remove(text_str.begin(), text_str.end(), '\r'), text_str.end() );
  text_str.erase(std::remove(text_str.begin(), text_str.end(), '\n'), text_str.end());

  std::string is_robot;
  if (msg->deputy_name == "extra"){
    is_robot ="0";
  }else{
    is_robot ="1";
  }
  std::stringstream ss;
  ss << is_robot << secs_100 << text_str.substr (0,48) ;
  // BOOLTIMEMESSAGE
  
  string msg_plus = ss.str() ;
  it->second.push_back (msg_plus) ;

  //outputBuffer(it->second);

}




int main(int argc, char ** argv) {

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "printfHandler Ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}


// if of size 3:
// push_back  ... puts in [2] and removes [0]
// pu

//boost circular buffer
// signal tap.cpp
