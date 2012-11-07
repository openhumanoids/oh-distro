// Selective ros2lcm translator
// mfallon aug,sept 2012

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>


#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>

#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <pr2_controllers_msgs/JointControllerState.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>

//#include <gazebo_msgs/ContactState.h>
//#include <gazebo_msgs/ContactsState.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>


#include <Eigen/Dense>
#include <Eigen/StdVector>
#define VERBOSE false
using namespace std;
//using namespace cv;

class App{
public:
  App(const std::string & stereo_in,
      const std::string & stereo_out);
  ~App();

private:
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;

  // Left and Right Images Seperately:
  void left_image_cb(const pr2_controllers_msgs::JointControllerStateConstPtr& msg);
  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);

  void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber left_image_sub_,right_image_sub_, joint_states_sub_;
  void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );

  // Stereo Image:
  const std::string stereo_in_,stereo_out_;
  int counter;
};

App::App(const std::string & stereo_in,
    const std::string & stereo_out):
    stereo_in_(stereo_in),
    stereo_out_(stereo_out){
  ROS_INFO("Initializing Translator");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  left_image_sub_ = node_.subscribe(string("/head_hokuyo_joint_position_controller/state"), 10, &App::left_image_cb,this);

  joint_states_sub_ = node_.subscribe(string("/joint_states"), 10, &App::joint_states_cb,this);

  
//  right_image_sub_ = node_.subscribe(string("/right_eye/image_raw"), 10, &App::right_image_cb,this);
  //left_image_sub_ = node_.subscribe(string("/left_eye/image_rect_color"), 10, &App::left_image_cb,this);
  //right_image_sub_ = node_.subscribe(string("right_eye/image_rect_color"), 10, &App::right_image_cb,this);


  counter=0;
};

App::~App()  {
}

int counter=0;




Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}


void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  std::cout << "head_hokuyo_joint_position_controller\n";
  
  for(size_t i=0; i< msg->name.size(); i++){
     std::cout << "i: " << i << msg->name[i] <<  "\n"; 
     if (  msg->name[i].compare("neck.ay") == 0){
       std::cout << "i: " << i << msg->name[i] <<  " | "
                 << msg->position[i] <<  " | "
                 << msg->velocity[i] <<  "\n"; 
       
       
	Eigen::Quaterniond r= euler_to_quat(0,0, msg->position[i]);
    
    
	ros::Time current_time = msg->header.stamp;
	int64_t current_utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);    
    
	bot_core::rigid_transform_t tf;
	tf.utime = current_utime;
	tf.trans[0] = 0;
	tf.trans[1] = 0;
	tf.trans[2] = 0;
    
	tf.quat[0] = r.w();
	tf.quat[1] = r.x();
	tf.quat[2] = r.y();
	tf.quat[3] = r.z();

	lcm_publish_.publish("BODY_TO_ROTATING_SCAN", &tf);
       
       
       return;
     }
  }  
}


void App::left_image_cb(const pr2_controllers_msgs::JointControllerStateConstPtr& msg){
  std::cout << msg->process_value << " head_hokuyo_joint_position_controller\n";
    
  Eigen::Quaterniond r= euler_to_quat(0,0, msg->process_value);
    
    
  ros::Time current_time = msg->header.stamp;
  int64_t current_utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);    
    
  bot_core::rigid_transform_t tf;
  tf.utime = current_utime;
  tf.trans[0] = 0;
  tf.trans[1] = 0;
  tf.trans[2] = 0;
    
  tf.quat[0] = r.w();
  tf.quat[1] = r.x();
  tf.quat[2] = r.y();
  tf.quat[3] = r.z();

  lcm_publish_.publish("BODY_TO_ROTATING_SCAN", &tf);
}


int main(int argc, char **argv){
  std::cout << "ros2lcm_translator_combined launched\n";
  ros::init(argc, argv, "ros2lcm_translator_combined");

  App *app = new App("camera", "CAMERA");

  ros::spin();
  return 0;
}
