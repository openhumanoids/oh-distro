#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <boost/shared_ptr.hpp>

#include "laser.h"
#include "camera.h"
#include "imu.h"

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

using namespace crl::multisense;
Channel *d;
multisense_ros::Laser  *l;
multisense_ros::Camera *camera ;
multisense_ros::Imu    *imuObj;

bool signal_caught = false;

void signal_handler(int s){
  if (signal_caught) return;
  signal_caught = true;
  printf("Caught signal, stopping driver\n");
  delete l;
  delete camera;
  delete imuObj;
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
    //void requestHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
    //                    const  drc::sensor_request_t* msg);   
    
    void commandHandler(const lcm::ReceiveBuffer* rbuf, 
                          const std::string& channel, const  multisense::command_t* msg);

};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){
  //lcm_->subscribe( "SENSOR_REQUEST" ,&Pass::requestHandler,this);
  
  lcm_->subscribe( "MULTISENSE_COMMAND" ,&Pass::commandHandler,this);  
}

/*
void Pass::requestHandler(const lcm::ReceiveBuffer* rbuf, 
                          const std::string& channel, const  drc::sensor_request_t* msg){
   std::cout << "Request Message Received\n";
   CameraConfig config;
   config.spindle_rpm_ = (float) msg->spindle_rpm;
   
   config.fps_ = (float) msg->head_fps;
   camera->applyConfig(config);
} */

void Pass::commandHandler(const lcm::ReceiveBuffer* rbuf, 
                          const std::string& channel, const  multisense::command_t* msg){
   std::cout << "Command Message Received\n";
   CameraConfig config;
   config.spindle_rpm_ = (float) msg->rpm;
   config.fps_ = (float) msg->fps;
   config.gain_ = (float) msg->gain;
   config.agc_ = msg->agc;
   config.leds_flash_ = msg->leds_flash;
   config.leds_duty_cycle_ = msg->leds_duty_cycle;
   camera->applyConfig(config);
}



int main(int    argc, char** argv){
  CameraConfig config;
  ConciseArgs opt(argc, (char**)argv);
  bool use_imu = false;
  opt.add(config.do_jpeg_compress_, "j", "do_jpeg_compress","Do JPEG compression");
  opt.add(config.do_zlib_compress_, "z", "do_zlib_compress","Do Z compression");
  opt.add(config.output_mode_, "o", "output mode","modes: 0leftcolor&depth, 1leftgrey&rightgrey, 2leftcolor, 3leftcolor&depth&rightgray");
  opt.add(config.desired_width_, "w", "desired_width", "Desired image width");
  opt.add(use_imu, "i", "use_imu", "Publish IMU data");
  opt.parse();
  
  
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
    camera = new multisense_ros::Camera(d, config );
    imuObj = use_imu ? new multisense_ros::Imu(d) : NULL;
    
    while(1){
      // This is what I'm most unsure about in this driver:
      nanosleep((struct timespec[]){{0, 10000000}}, NULL);
      lcm->handle();
    }
  }

  Channel::Destroy(d);

  return 0;
}
