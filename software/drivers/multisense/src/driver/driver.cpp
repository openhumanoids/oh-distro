#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <boost/shared_ptr.hpp>

#include "laser.h"
#include "camera.h"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace crl::multisense;
Channel *d;
multisense_ros::Laser  *l;
multisense_ros::Camera *camera ;

void signal_handler(int s){
  printf("Caught signal, stopping driver\n");
  camera->~Camera();
  Channel::Destroy(d);
  exit(1); 
}

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void requestHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                        const  drc::sensor_request_t* msg);   

};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){
  lcm_->subscribe( "SENSOR_REQUEST" ,&Pass::requestHandler,this);
}

void Pass::requestHandler(const lcm::ReceiveBuffer* rbuf, 
                          const std::string& channel, const  drc::sensor_request_t* msg){
   std::cout << "Config Message Received\n";
   CameraConfig config;
   config.spindle_rpm = (float) msg->spindle_rpm;
   
   config.fps = (float) msg->head_fps;
   camera->applyConfig(config);
}



int main(int    argc, char** argv){
  
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);   

  std::string robot_desc_string;
  std::string sensor_ip = "10.66.171.21";
  int         sensor_mtu = 7200;

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }    
    
  Pass app(lcm);
    
  d= Channel::Create(sensor_ip);
  if (d) {

    if (Status_Ok != d->setMtu(sensor_mtu))
        printf("failed to set sensor MTU to %d", sensor_mtu);

    l = new multisense_ros::Laser(d, robot_desc_string);
    camera = new multisense_ros::Camera(d);
    
    while(1){
      // This is what I'm most unsure about in this driver:
      nanosleep((struct timespec[]){{0, 10000000}}, NULL);
      lcm->handle();
    }
  }

  Channel::Destroy(d);

  return 0;
}
