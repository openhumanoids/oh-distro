#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <path_util/path_util.h>
#include <lcmtypes/bot_core.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmtypes/bot_param/set_t.hpp>
#include <lcmtypes/bot_param/entry_t.hpp>
#include <lcmtypes/bot_param/update_t.hpp>

#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <ConciseArgs>

////////////////////////////////////////
struct CommandLineConfig
{
    bool mode;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_);
    
    ~App(){
    }    
    
    void sendUpdate();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig& ca_cfg_;
    
    void paramsHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_param::update_t* msg);   
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    
    int counter_; // used for terminal feedback
    int verbose_;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_){
      if (ca_cfg_.mode == true){
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
      }
  
  counter_ =0;  
  verbose_=3; // 1 important, 2 useful 3, lots
  
  lcm_->subscribe("PARAM_UPDATE",&App::paramsHandler,this);  
}

void App::sendUpdate(){
  
  std::cout << "Updating\n";
  
  bot_param::set_t msg;
  msg.utime = bot_timestamp_now();
  msg.sequence_number = bot_param_get_seqno(botparam_);
  msg.server_id = bot_param_get_server_id(botparam_);

  bot_param::entry_t entry;
  entry.is_array = true;
  std::string key = "control.encoder_offsets.index";
  entry.key = key;
  entry.value = "1,2,3,4,5,6,7,8,9,10,11,12";
  char* val = NULL;
  if (bot_param_get_str(botparam_, key.c_str(), &val) != 0){
    std::cout << "couldn't find key\n"; 
  }else{
    msg.entries.push_back(entry);
  }
  

  bot_param::entry_t entry2;
  entry2.is_array = true;
  key = "control.encoder_offsets.value";
  entry2.key = key;
  entry2.value = ".01,.02,.03,.04,.05,.06,.07,.08,.09,.10,.11,.12";
  char* val2 = NULL;
  if (bot_param_get_str(botparam_, key.c_str(), &val2) != 0){
    std::cout << "couldn't find key\n"; 
  }else{
    msg.entries.push_back(entry2);
  }
  
  
  
  msg.numEntries = msg.entries.size();
  lcm_->publish("PARAM_SET", &msg);
  
  
}


void App::paramsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_param::update_t* msg){
  std::cout << "\n" << msg->params << "\n";
}


int main(int argc, char ** argv) {
  CommandLineConfig ca_cfg;
  ca_cfg.mode = 0;
 
  ConciseArgs opt(argc, (char**)argv);
  opt.add(ca_cfg.mode , "m", "type","Mode: 0 spit params. 1 push");
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App accu(lcm, ca_cfg);
  
  if (ca_cfg.mode == 0){
    while(0 == lcm->handle());  
  }else{
    accu.sendUpdate();
  }

  return 0;
}
