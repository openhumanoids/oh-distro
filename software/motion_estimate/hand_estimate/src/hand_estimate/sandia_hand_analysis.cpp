#include <stdio.h>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <ConciseArgs>
#include <string>
#include <deque>
using namespace std;


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left);
    
    ~App(){
    }
    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    bool is_left;
    
    std::vector < float > command_pos_;
    std::vector<float> sensed_pos_;
    
    //////////////////////////////////////
    void commandHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::joint_command_t* msg);    
    
    void rawHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::raw_sandia_hand_t* msg);      
    
    void stateHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::hand_state_t* msg); 
   
    float logit(float x, float b0=0, float b1=1);
};   

App::App(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left):
   lcm_(lcm_), is_left(is_left){
  verbose_=false;
  
  lcm_->subscribe(is_left ? "L_HAND_JOINT_COMMANDS":"R_HAND_JOINT_COMMANDS", 
                                        &App::commandHandler, this);  

  lcm_->subscribe(is_left ? "SANDIA_LEFT_RAW":"SANDIA_RIGHT_RAW", 
                                        &App::rawHandler, this);  
  
  lcm_->subscribe(is_left ? "SANDIA_LEFT_STATE":"SANDIA_RIGHT_STATE", 
                                        &App::stateHandler, this);  
  
}

void App::commandHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::joint_command_t* msg){
  
  std::cout << "got command\n";
  std::vector<float> v_float( msg->position.begin(),  msg->position.end());  
  command_pos_ = v_float;  
}

void App::rawHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::raw_sandia_hand_t* msg){
  lcm_->publish("SANDIA_LEFT_RAW_MOBO", &msg->mobo);
}

// logistic function F(x)=1/(1+e^{-b_1 (x-b_0)})
float App::logit(float x, float b0, float b1){
  return 1.0/(1.0+exp(-(b1*(x-b0))));
}

void App::stateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::hand_state_t* msg){
  
  sensed_pos_ = msg->joint_position;
  // std::cout << "got state\n";
  
  
  if (command_pos_.size() != sensed_pos_.size()){
    //std::cout << "sensed and commanded don't match\n";
    return; 
  }
  
  std::vector <float> diffs;
  for (size_t i=0; i < command_pos_.size() ; i++){
    diffs.push_back( fabs( command_pos_[i] - sensed_pos_[i] ) );
  }
  std::vector <float> finger_diffs(4,0.0);
  finger_diffs[0] = diffs[0] + diffs[1] + diffs[2];
  finger_diffs[1] = diffs[3] + diffs[4] + diffs[5];
  finger_diffs[2] = diffs[6] + diffs[7] + diffs[8];
  finger_diffs[3] = diffs[9] + diffs[10] + diffs[11];
  
  
  std::cout << diffs[9] << " " << diffs[10] << " " << diffs[11] << "\n";
  
  if (finger_diffs[3] > 0.2){
    std::cout << finger_diffs[3] << " BIG==================\n"; 
  }else{
    std::cout << finger_diffs[3] << "SMALL\n"; 
  }
  
  std::cout << "logit(finger_diffs[0])=" << logit(finger_diffs[0],0.1,60) << endl;
  std::cout << "logit(finger_diffs[1])=" << logit(finger_diffs[1],0.1,60) << endl;
  std::cout << "logit(finger_diffs[2])=" << logit(finger_diffs[2],0.1,60) << endl;
  std::cout << "logit(finger_diffs[3])=" << logit(finger_diffs[3],0.1,60) << endl;
  
}


 
int main(int argc, char *argv[]){
  bool lhand=false, rhand=false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(rhand, "r", "right","Process right hand message");
  opt.add(lhand, "l", "left","Process left hand message");
  
  opt.parse();
  if(rhand && lhand){
    printf("Only one hand at a time.\n");
    return 1;
  }
  if(!rhand && !lhand){
    printf("Please specify a hand. Type -h for usage.\n");
    return 1;
  }
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  App app(lcm, lhand);
  while(0 == lcm->handle());
  return 0;
}
