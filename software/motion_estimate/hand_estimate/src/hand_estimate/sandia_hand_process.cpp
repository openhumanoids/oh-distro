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
    bool is_left;
    string handtext;  // Put RIGHT / LEFT
    
    // For calibration of tactile sensor
    unsigned int init_sandia_cnt;
    static const unsigned int SANDIA_CALIB_NUM = 200;
    static const unsigned int NTACTILE = 32;
    float sandia_tactile_offset[NTACTILE];
    float sandia_tactile_processed[NTACTILE];
    static const int TACTILE_THRESHOLD = 3000;
    //////////////////////////////////////
    void sandiaRawHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::raw_sandia_hand_t* msg);    
    
    void publishSandiaTactile(int64_t utime, float palm_tactile[]);
};   

App::App(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left):
   lcm_(lcm_), is_left(is_left){
  
  init_sandia_cnt = 0;
  handtext = is_left? "LEFT": "RIGHT";
  
  for(size_t i=0; i<NTACTILE; i++)
    sandia_tactile_offset[i] = 0;
  printf("Start processing %s hand.\n", handtext.c_str());
  
  lcm_->subscribe(is_left ? "SANDIA_LEFT_RAW":"SANDIA_RIGHT_RAW", 
                                        &App::sandiaRawHandler, this);  
}

void App::sandiaRawHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::raw_sandia_hand_t* msg){
  
  // tactile calibration
  if(init_sandia_cnt < SANDIA_CALIB_NUM){
    for(size_t i=0; i < NTACTILE; i++)
      sandia_tactile_offset[i] += msg->palm.palm_tactile[i] / (float)SANDIA_CALIB_NUM;
    if(init_sandia_cnt == SANDIA_CALIB_NUM-1)
      printf("Sandia %s hand palm tactile calibration done.\n", handtext.c_str());

    init_sandia_cnt++;
  }
  else
  {
    for(size_t i=0;i<NTACTILE;i++)
      sandia_tactile_processed[i] = msg->palm.palm_tactile[i] - sandia_tactile_offset[i];
    publishSandiaTactile(msg->utime, sandia_tactile_processed);
  }
  ///////////////////////
  
  
}
 
void App::publishSandiaTactile(int64_t utime, float palm_tactile[]){
  drc::hand_tactile_state_t msg_out;
  msg_out.utime = utime; 

  msg_out.n_f0 = 0;
  msg_out.n_f1 = 0;
  msg_out.n_f2 = 0;
  msg_out.n_f3 = 0;
  
  // offset the raw signals
  msg_out.n_palm = NTACTILE;
  msg_out.palm.resize(NTACTILE);
  for(size_t i=0; i<NTACTILE; i++){
    msg_out.palm[i] = (float)palm_tactile[i];
  }
  
  // summarize signals
  float vmax = palm_tactile[0], vmin = palm_tactile[0];
  for(size_t i = 1; i < NTACTILE; i++){
    float v = palm_tactile[i];
    vmax = std::max(vmax,v);
    vmin = std::min(vmin,v);
  }
  
  msg_out.signal = (float)fabs(vmax-vmin); 
  msg_out.touched =  msg_out.signal > TACTILE_THRESHOLD;
  lcm_->publish("SANDIA_"+handtext+"_TACTILE_STATE", &msg_out); 
  
  // for gnuplot
  static deque<float> dq;
  dq.push_back(msg_out.signal);
  if(dq.size()>500)
    dq.pop_front();
  printf("plot \"-\" with lines\n");
  for(int i=0;i<dq.size();i++)
    printf("%f\n", dq[i]);
  printf("e\n");
  printf("set yrange [0:60000]\n");
  printf("set xrange [0:600]\n");
  /////////////////////////////
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
