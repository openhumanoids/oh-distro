#include <stdio.h>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <ConciseArgs>
#include <string>
#include <deque>
using namespace std;

enum HandType {SANDIA, IROBOT};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left, bool use_gnuplot_, HandType ht);
    
    ~App(){}
    
  protected:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool is_left;
    string strHandLR;  // Put RIGHT / LEFT
    string strHandName;  // Put IROBOT / SANDIA
    bool use_gnuplot_;
    
    float logit(float x, float b0=0, float b1=1);
};  

class AppSandia: public App{
  public:
    AppSandia(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left, bool use_gnuplot_);
    
    ~AppSandia(){}
    
    
  private:
    // For calibration of tactile sensor
    unsigned int init_sandia_cnt;
    static const unsigned int SANDIA_CALIB_NUM = 200;
    static const unsigned int NTACTILE = 32;
    float sandia_tactile_offset[NTACTILE];
    float sandia_tactile_processed[NTACTILE];
    float prob_sandia_tactile_processed[NTACTILE];
    static const int TACTILE_THRESHOLD = 4000;
    //////////////////////////////////////
    void sandiaRawHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::raw_sandia_hand_t* msg);    
    
    void publishSandiaTactile(int64_t utime);
    
    
    ////// For finger analysis
    std::vector < float > command_pos_;
    std::vector<float> sensed_pos_;
    static const unsigned int NFINGER = 4;
    float finger_tactile[NFINGER];
    float prob_finger_tactile[NFINGER];
    /////////////////////////////////////////
    void commandHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::joint_command_t* msg);    
    void stateHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::hand_state_t* msg); 
};  

class AppIRobot: public App{
  public:
    AppIRobot(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left, bool use_gnuplot_);
    
    ~AppIRobot(){}
    
    
  private:
    static const unsigned int NTACTILE = 48;
    static const int TACTILE_THRESHOLD = 100;
    float sandia_tactile_processed[NTACTILE];
    float prob_sandia_tactile_processed[NTACTILE];
    //////////////////////////////////////
    void irobotHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::raw_irobot_hand_t* msg);    
    
    void publishIRobotTactile(int64_t utime);
    
    
    ////// todo: For finger analysis 
    //static const unsigned int NFINGER = 4;
    //float finger_tactile[NFINGER];
    //float prob_finger_tactile[NFINGER];
    /////////////////////////////////////////
};  

App::App(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left, bool use_gnuplot_, HandType ht):
   lcm_(lcm_), is_left(is_left), use_gnuplot_(use_gnuplot_){
  
  strHandLR = is_left? "LEFT": "RIGHT";
  strHandName = ht==SANDIA? "SANDIA": "IROBOT";
  printf("Start processing %s %s hand.\n", strHandName.c_str(), strHandLR.c_str());
}

AppSandia::AppSandia(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left, bool use_gnuplot_):
   App(lcm_, is_left, use_gnuplot_, SANDIA) {
  
  init_sandia_cnt = 0;
  
  for(size_t i=0; i<NTACTILE; i++)
    sandia_tactile_offset[i] = 0;
  
  // for fingers
  lcm_->subscribe(is_left ? "L_HAND_JOINT_COMMANDS":"R_HAND_JOINT_COMMANDS", 
                                        &AppSandia::commandHandler, this);  
  lcm_->subscribe(is_left ? "SANDIA_LEFT_STATE":"SANDIA_RIGHT_STATE", 
                                        &AppSandia::stateHandler, this); 
  // for palm
  lcm_->subscribe(is_left ? "SANDIA_LEFT_RAW":"SANDIA_RIGHT_RAW", 
                                        &AppSandia::sandiaRawHandler, this);  
}

AppIRobot::AppIRobot(boost::shared_ptr<lcm::LCM> &lcm_, bool is_left, bool use_gnuplot_):
   App(lcm_, is_left, use_gnuplot_, IROBOT) {
     
  // for palm
  lcm_->subscribe(is_left ? "IROBOT_LEFT_RAW":"IROBOT_RIGHT_RAW", 
                                        &AppIRobot::irobotHandler, this);  
}


// logistic function F(x)=1/(1+e^{-b_1 (x-b_0)})
float App::logit(float x, float b0, float b1){
  return 1.0/(1.0+exp(-(b1*(x-b0))));
}

void AppSandia::sandiaRawHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::raw_sandia_hand_t* msg){
  
  // tactile calibration
  if(init_sandia_cnt < SANDIA_CALIB_NUM){
    for(size_t i=0; i < NTACTILE; i++)
      sandia_tactile_offset[i] += msg->palm.palm_tactile[i] / (float)SANDIA_CALIB_NUM;
    if(init_sandia_cnt == SANDIA_CALIB_NUM-1)
      printf("Sandia %s hand palm tactile calibration done.\n", strHandLR.c_str());

    init_sandia_cnt++;
  }
  else
  {
    for(size_t i=0;i<NTACTILE;i++)
      sandia_tactile_processed[i] = msg->palm.palm_tactile[i] - sandia_tactile_offset[i];
    
    // sensor 28 of left hand respond in an opposite way
    if(is_left)
      sandia_tactile_processed[28] *= -1;
      
    // map 0-65536 to 0-1 using logistic function
    for(size_t i=0; i<NTACTILE; i++)
      prob_sandia_tactile_processed[i] = logit(sandia_tactile_processed[i], 3000, 0.005);
    publishSandiaTactile(msg->utime);
  }
  ///////////////////////
}

void AppIRobot::irobotHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::raw_irobot_hand_t* msg){
  
  // no tactile calibration needed
  for(size_t i=0;i<NTACTILE;i++)
    sandia_tactile_processed[i] = msg->palmTactile[i];
      
  // map 0-200 to 0-1 using logistic function
  for(size_t i=0; i<NTACTILE; i++){
    prob_sandia_tactile_processed[i] = logit(sandia_tactile_processed[i], 100, 0.1);
  }
  publishIRobotTactile(msg->utime);
}

void AppSandia::publishSandiaTactile(int64_t utime){
  drc::hand_tactile_state_t msg_out;
  msg_out.utime = utime; 

  msg_out.n_palm = NTACTILE;
  msg_out.palm.resize(NTACTILE);
  for(size_t i=0; i<NTACTILE; i++){
    //msg_out.palm[i] = (float)sandia_tactile_processed[i];
    msg_out.palm[i] = (float)prob_sandia_tactile_processed[i];
  }
  
  // summarize signals
  float vmax = sandia_tactile_processed[0], vmin = sandia_tactile_processed[0];
  float vsum = 0;
  for(size_t i = 1; i < NTACTILE; i++){
    float v = sandia_tactile_processed[i];
    vmax = std::max(vmax,v);
    vmin = std::min(vmin,v);
    
    vsum += v;
  }
  
  ////fingers
  msg_out.n_f0 = 1;
  msg_out.n_f1 = 1;
  msg_out.n_f2 = 1;
  msg_out.n_f3 = 1;
  msg_out.f0.push_back(prob_finger_tactile[0]);
  msg_out.f1.push_back(prob_finger_tactile[1]);
  msg_out.f2.push_back(prob_finger_tactile[2]);
  msg_out.f3.push_back(prob_finger_tactile[3]);
  
  
  msg_out.signal = logit(vsum, TACTILE_THRESHOLD, 0.1); //(float)fabs(vmax-vmin); 
  msg_out.touched =  vsum > TACTILE_THRESHOLD;
  lcm_->publish(strHandName+"_"+strHandLR+"_TACTILE_STATE", &msg_out);   
  
  
  if (use_gnuplot_){// for gnuplot
    static deque<float> dq;
    dq.push_back(msg_out.signal);
    if(dq.size()>500)
      dq.pop_front();
    printf("plot \"-\" with lines\n");
    for(size_t i=0;i<dq.size();i++)
      printf("%f\n", dq[i]);
    printf("e\n");
    printf("set yrange [-10000:30000]\n");
    printf("set xrange [0:600]\n");
    /////////////////////////////
  }
  
  
  
}
void AppIRobot::publishIRobotTactile(int64_t utime){
  drc::hand_tactile_state_t msg_out;
  msg_out.utime = utime; 

  msg_out.n_palm = NTACTILE;
  msg_out.palm.resize(NTACTILE);
  for(size_t i=0; i<NTACTILE; i++){
    //msg_out.palm[i] = (float)sandia_tactile_processed[i];
    msg_out.palm[i] = (float)prob_sandia_tactile_processed[i];
  }
  
  // summarize signals
  float vmax = sandia_tactile_processed[0], vmin = sandia_tactile_processed[0];
  float vsum = 0;
  for(size_t i = 1; i < NTACTILE; i++){
    float v = sandia_tactile_processed[i];
    vmax = std::max(vmax,v);
    vmin = std::min(vmin,v);
    
    vsum += v;
  }
  
  ////fingers
  msg_out.n_f0 = 0;
  msg_out.n_f1 = 0;
  msg_out.n_f2 = 0;
  msg_out.n_f3 = 0;
  //msg_out.f0.push_back(prob_finger_tactile[0]);
  //msg_out.f1.push_back(prob_finger_tactile[1]);
  //msg_out.f2.push_back(prob_finger_tactile[2]);
  //msg_out.f3.push_back(prob_finger_tactile[3]);
  
  
  msg_out.signal = logit(vsum, TACTILE_THRESHOLD, 0.1); //(float)fabs(vmax-vmin); 
  msg_out.touched =  vsum > TACTILE_THRESHOLD;
  lcm_->publish(strHandName+"_"+strHandLR+"_TACTILE_STATE", &msg_out); 
  
  
  if (use_gnuplot_){// for gnuplot
    static deque<float> dq;
    dq.push_back(msg_out.signal);
    if(dq.size()>100)
      dq.pop_front();
    printf("plot \"-\" with lines\n");
    for(size_t i=0;i<dq.size();i++)
      printf("%f\n", dq[i]);
    printf("e\n");
    printf("set yrange [-5000:5000]\n");
    printf("set xrange [0:600]\n");
    /////////////////////////////
  }
  
}

void AppSandia::commandHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::joint_command_t* msg){
  
  //std::cout << "got command\n";
  std::vector<float> v_float( msg->position.begin(),  msg->position.end());  
  command_pos_ = v_float;  
}

void AppSandia::stateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::hand_state_t* msg){
  
  sensed_pos_ = msg->joint_position;
  //std::cout << "got state\n";
  
  
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
  
  
  //std::cout << diffs[9] << " " << diffs[10] << " " << diffs[11] << "\n";
  
  if (finger_diffs[3] > 0.2){
    //std::cout << finger_diffs[3] << " BIG==================\n"; 
  }else{
    //std::cout << finger_diffs[3] << "SMALL\n"; 
  }
  
  for(size_t i=0; i<NFINGER; i++){
    finger_tactile[i] = finger_diffs[i];
    prob_finger_tactile[i] = logit(finger_diffs[i],0.1,60);
    //std::cout << "logit(finger_diffs["<<i<<"])=" << logit(finger_diffs[0],0.1,60) << endl;
  }
}


int main(int argc, char *argv[]){
  bool use_gnuplot=false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(use_gnuplot, "g", "use_gnuplot","Verbose printf, to pipe to gnuplot append '| gnuplot'");
  opt.parse();
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  AppSandia app_sandia_l(lcm, 1, 0);
  AppSandia app_sandia_r(lcm, 0, 0);
  AppIRobot app_irobot_l(lcm, 1, 0);
  AppIRobot app_irobot_r(lcm, 0, 0);
  
  while(0 == lcm->handle());
  return 0;
}
