#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <bot_core/bot_core.h>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pointcloud_tools/pointcloud_math.hpp>

#include <ConciseArgs>

using namespace std;


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
        
  private:
    int verbose_;
    int64_t last_utime_;
    boost::shared_ptr<lcm::LCM> lcm_;
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): 
    lcm_(lcm_){
  
  lcm_->subscribe( "EST_ROBOT_STATE" ,&Pass::robotStateHandler,this);
  last_utime_ =0;
}

void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::robot_state_t* msg){
  
  if (msg->utime < 10E6){
    std::cout << "skip start of log\n";
    return;
  }
  Eigen::Quaterniond quat_in = Eigen::Quaterniond( msg->origin_position.rotation.w, msg->origin_position.rotation.x,
                    msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  double ypr[3];
  quat_to_euler(quat_in, ypr[0], ypr[1], ypr[2]);
  
  double thres = 40*M_PI/180;
  
  if (ypr[2] > thres   || ypr[1] > thres ) {
    std::cout << "Pitch: " << ypr[1]*180/M_PI << " | Roll: " << ypr[2]*180/M_PI << " Degrees\n";
    std::cout << "Fall Detected at sim time: "<< msg->utime << "\n";
    exit(-1);
  }
  
  if (msg->utime -  last_utime_  > 10E6){
    std::cout <<  msg->utime*1E-6 << " EST_ROBOT_STATE\n";
    last_utime_ = msg->utime;
  }
}


int main(int argc, char ** argv) {
  string filename = "path/to/lcm/log";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(filename, "f", "filename","Filename");
  opt.parse();
  std::cout << "filename: " << filename << " filename\n";      

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM(  string("file://" + filename + "?speed=0")  )  );
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Pin point fall Ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
