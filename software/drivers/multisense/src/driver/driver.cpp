/**
 * @file ros_driver.cpp
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "laser.h"
#include "camera.h"

#include <lcmtypes/drc_lcmtypes.hpp>

using namespace crl::multisense;
Channel *d;
multisense_ros::Laser  *l;
multisense_ros::Camera *camera ;

void my_handler(int s){
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
    void requestHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::sensor_request_t* msg);   

};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){

  lcm_->subscribe( "SENSOR_REQUEST" ,&Pass::requestHandler,this);
  
}

void Pass::requestHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::sensor_request_t* msg){
   std::cout << "got request\n";
   CameraConfig config;
   config.spindle_rpm = (float) msg->spindle_rpm;
   
   config.fps = (float) msg->head_fps;
   camera->applyConfig(config);
}



int main(int    argc, 
         char** argv)
{
  
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);   

    //
    // Get parameters from ROS/command-line

    std::string robot_desc_string;
    std::string sensor_ip;
    int         sensor_mtu;

    sensor_ip = "10.66.171.21";
    sensor_mtu = 7200;

    
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }    
    
   Pass app(lcm);
    
    {

        
        d= Channel::Create(sensor_ip);
    
	if (d) {

            if (Status_Ok != d->setMtu(sensor_mtu))
                printf("failed to set sensor MTU to %d", sensor_mtu);

            l = new multisense_ros::Laser(d, robot_desc_string);
            camera = new multisense_ros::Camera(d);
            
            while(1){
              nanosleep((struct timespec[]){{0, 10000000}}, NULL);
              lcm->handle();

            }
//            ros::spin();
        }

        Channel::Destroy(d);
    }

    return 0;
}
