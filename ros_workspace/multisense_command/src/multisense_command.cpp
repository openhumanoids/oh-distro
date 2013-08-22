// command line dynamic reconfigure:
// rosrun dynamic_reconfigure dynparam set /multisense_sl motor_speed 1
// rosrun dynamic_reconfigure dynparam set /multisense_sl fps 30
// C++ control of dynamic reconfigure is not implemented - need to use system call:
// http://ros.org/wiki/hokuyo_node/Tutorials/UsingDynparamToChangeHokuyoLaserParameters#PythonAPI

#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <std_msgs/Float64.h>

#include <ConciseArgs>

using namespace std;

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~App() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    lcm::LCM lcm_publish_ ;

    void sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg);
    ros::NodeHandle* rosnode;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  /// Spinning Laser control:
  lcm_->subscribe("SENSOR_REQUEST",&App::sensor_request_Callback,this);
  rosnode = new ros::NodeHandle();
}

void App::sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg){
  if ((msg->spindle_rpm >=0) && (msg->spindle_rpm <=49) ){ // driver sets max at 5.2rpm
    double spindle_rads = msg->spindle_rpm * (2*M_PI)/60; // convert from RPM to rad/sec
    std::stringstream ss;
    ss << "rosrun dynamic_reconfigure dynparam set /multisense_sl motor_speed " << spindle_rads;
    //ROS_ERROR("[%s]", ss.str().c_str() );    
    system( ss.str().c_str() );
    ROS_ERROR("App Setting Spindle RPM: %d", ((int) msg->spindle_rpm) );
    ROS_ERROR("                    rad/sec: %f", spindle_rads );
  }else {
    ROS_ERROR("App Ignoring Out of Range Spindle RPM: %d", ((int) msg->spindle_rpm) );
  }
    
  if ((msg->head_fps >=0) && (msg->head_fps <=30) ){ // driver sets minimum at 1fps
    std::stringstream ss;
    ss << "rosrun dynamic_reconfigure dynparam set /multisense_sl fps " << (int) msg->head_fps;
    //ROS_ERROR("[%s]", ss.str().c_str() );    
    system( ss.str().c_str() );
    ROS_ERROR("App Setting Head Camera FPS: %d", ((int) msg->head_fps) );
  }else {
    ROS_ERROR("App Ignoring Negative Head FPS: %d", ((int) msg->head_fps) );
  }
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"multisense_command",ros::init_options::NoSigintHandler);

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  App handlerObject(lcm, nh);
  ROS_ERROR("Multisense Command Ready");
  
  while(0 == lcm->handle());
  return 0;
}
