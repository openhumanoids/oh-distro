// Selective ros2lcm translator
//
// mfallon aug,sept 2012
#include "cv.h"
#include "highgui.h"


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <atlas_gazebo_msgs/RobotState.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#define VERBOSE false
using namespace std;
//using namespace cv;

class App{
public:
  App();
  ~App();

private:
  lcm::LCM lcm_publish_ ;

  ros::NodeHandle node_;

  //    uint8_t* image_data;

  ros::Subscriber base_scan_sub_,rstate_sub_;
  void base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void rstate_cb(const atlas_gazebo_msgs::RobotState::ConstPtr& msg);
  //    ros::Subscriber clock_sub_,left_image_sub_,right_image_sub_,points_sub_;
  //    void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  //    void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  //    void points_cb(const sensor_msgs::PointCloud2ConstPtr& msg);


  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  /*
    void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );
    void send_points(const sensor_msgs::PointCloud2ConstPtr& msg,string channel );
    int points_cb_counter; // temp used to skip frames of points
   */
};

App::App(){
//  lcm::LCM lcm_publish2_;
//  (lcm_publish_) = new lcm::LCM;//&lcm_publish2_;
  if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm.good()  returned false in TfPublisher::publishTransforms" <<std::endl;
  }

  //  tf_sub_ = node_.subscribe(string("/tf"), 10, &App::tf_cb,this);
  base_scan_sub_ = node_.subscribe(string("/base_scan"), 10, &App::base_scan_cb,this);

  rstate_sub_ = node_.subscribe("true_robot_state", 10, &App::rstate_cb,this);



  //  left_image_sub_ = node_.subscribe(string("/wide_stereo/left/image_rect_color"), 10, &App::left_image_cb,this);
  //  right_image_sub_ = node_.subscribe(string("/wide_stereo/right/image_rect_color"), 10, &App::right_image_cb,this);
};

App::~App()  {
}

/*
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

void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}
 */


// typical contents: 640 x 480 images
// points: xyz
// channels: rgb u v
//void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
//  send_image(msg, "WIDE_LEFT_RECT");
//}
//void App::right_image_cb(const sensor_msgs::ImageConstPtr& msg){
//  send_image(msg, "WIDE_RIGHT_RECT");
//}



// TODO: jpeg compression:
//       look into openni_utils/openni_ros2rgb for code
// TODO: pre-allocate image_data and retain for speed - when size is known
/*void App::send_image(const sensor_msgs::ImageConstPtr& msg,string channel ){
  image_data = new uint8_t [640*480*3];
  IplImage *imageBGR = cvCreateImage(cvSize( msg->width ,   msg->height), 8, 3);
  IplImage* imageRGB = cvCreateImage(cvSize( msg->width ,   msg->height), 8, 3);

  copy(msg->data.begin(), msg->data.end(), image_data);
  imageBGR->imageData = (char*) image_data;
  cvCvtColor( imageBGR, imageRGB, CV_BGR2RGB );
  Mat imageRGBmat(imageRGB);

  bot_core_image_t img;
  img.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  img.width = msg->width;
  img.height =msg->height;
  img.row_stride = 3*msg->width; // check this
  img.size = msg->width*msg->height*3;
  img.nmetadata=0;
  img.metadata=NULL;
  img.data=imageRGBmat.data;
  img.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
  bot_core_image_t_publish(lcm_publish_,channel.c_str(), &img);

  if (VERBOSE){
    cout << "got: " << channel<< "\n";
    cout << msg->width << ", " << msg->height << ", "
        << msg->encoding << ", " << ((int) msg->is_bigendian) << ", " << msg->step << "\n";
    //640, 480, mono8, 0, 640
    cout << img.utime << "\n";
  }
  cvReleaseImage(&imageBGR);
  cvReleaseImage(&imageRGB);
  delete[] image_data;
}*/


void App::rstate_cb(const atlas_gazebo_msgs::RobotState::ConstPtr& msg){
  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  robot_state_msg.robot_name = msg->robot_name;
  robot_state_msg.origin_position.translation.x = msg->body_pose.position.x;
  robot_state_msg.origin_position.translation.y = msg->body_pose.position.y;
  robot_state_msg.origin_position.translation.z = msg->body_pose.position.z;
  robot_state_msg.origin_position.rotation.x = msg->body_pose.orientation.x;
  robot_state_msg.origin_position.rotation.y = msg->body_pose.orientation.y;
  robot_state_msg.origin_position.rotation.z = msg->body_pose.orientation.z;
  robot_state_msg.origin_position.rotation.w = msg->body_pose.orientation.w;

  robot_state_msg.origin_twist.linear_velocity.x =msg->body_twist.linear.x;
  robot_state_msg.origin_twist.linear_velocity.y =msg->body_twist.linear.y;
  robot_state_msg.origin_twist.linear_velocity.z =msg->body_twist.linear.z;
  robot_state_msg.origin_twist.angular_velocity.x =msg->body_twist.angular.x;
  robot_state_msg.origin_twist.angular_velocity.y =msg->body_twist.angular.y;
  robot_state_msg.origin_twist.angular_velocity.z =msg->body_twist.angular.z;

  int i,j;
  for(i = 0; i < 6; i++)  {
    for(j = 0; j < 6; j++) {
      robot_state_msg.origin_cov.position_cov[i][j] = 0;
      robot_state_msg.origin_cov.twist_cov[i][j] = 0;
    }
  }

  robot_state_msg.num_joints = msg->joint_name.size();
  robot_state_msg.joint_name = msg->joint_name;

  drc::joint_covariance_t j_cov;
  j_cov.variance = 0;

  for (std::vector<int>::size_type i = 0; i != robot_state_msg.joint_name.size(); i++)  {
    robot_state_msg.joint_position.push_back(msg->joint_position[i]);
    robot_state_msg.joint_velocity.push_back(msg->joint_velocity[i]);
    robot_state_msg.measured_effort.push_back(0);
    robot_state_msg.joint_cov.push_back(j_cov);
  }


  // dummy ground contact states
  robot_state_msg.contacts.num_contacts =2;
  robot_state_msg.contacts.id.push_back("Left_Foot");
  robot_state_msg.contacts.id.push_back("Right_Foot");
  robot_state_msg.contacts.inContact.push_back(0);
  robot_state_msg.contacts.inContact.push_back(0);

  cout << "r\n";

  lcm_publish_.publish("TRUE_ROBOT_STATE", &robot_state_msg);
}




void App::base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "BASE_SCAN");
}

void App::send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core::planar_lidar_t scan_out;

  float range_line[msg->ranges.size()];
  for (size_t i=0; i < msg->ranges.size(); i++){
    scan_out.ranges.push_back( msg->ranges[i] );
    //range_line[i] = msg->ranges[i];
  }
  scan_out.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  scan_out.nranges =msg->ranges.size();
  //  scan_out.ranges=range_line;
  scan_out.nintensities=0;
  //scan_out.intensities=NULL;
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;

  cout << "l\n";
  lcm_publish_.publish(channel.c_str(), &scan_out);

  //  bot_core_planar_lidar_t_publish(lcm_publish_,channel.c_str(),&scan_out);
  if (VERBOSE) cout << channel << ": "<< scan_out.utime << "\n";
}


int main(int argc, char **argv){
  std::cout << "ros2lcm_translator launched\n";
  ros::init(argc, argv, "ros2lcm_translator");
  App *app = new App();
  ros::spin();
  return 0;
}
