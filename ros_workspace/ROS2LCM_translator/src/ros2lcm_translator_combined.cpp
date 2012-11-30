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

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <atlas_gazebo_msgs/RobotState.h>

#include <rosgraph_msgs/Clock.h>

//#include <gazebo_msgs/ContactState.h>
//#include <gazebo_msgs/ContactsState.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#define VERBOSE false
using namespace std;
//using namespace cv;


int width =320; // hardcoded
int height =240; // hardcoded
uint8_t* stereo_data = new uint8_t [2*3* width*height]; // 2 color scale images stacked
uint8_t* singleimage_data = new uint8_t [2*2* width*height]; // 1 color scale image

class App{
public:
  App(const std::string & stereo_in,
      const std::string & stereo_out);
  ~App();

private:
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Clock:
  ros::Subscriber clock_sub_;
  void clock_cb(const rosgraph_msgs::ClockConstPtr& msg);
  
  // Robot State:
  ros::Subscriber rstate_sub_;
  void rstate_cb(const atlas_gazebo_msgs::RobotStateConstPtr& msg);
  
  // Toe Sensors:
  ros::Subscriber LFootToeIn_cstate_sub_,LFootToeOut_cstate_sub_,LFootHeelIn_cstate_sub_,LFootHeelOut_cstate_sub_;
  ros::Subscriber RFootToeIn_cstate_sub_,RFootToeOut_cstate_sub_,RFootHeelIn_cstate_sub_,RFootHeelOut_cstate_sub_;
/*  void LFootToeIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void LFootToeOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void LFootHeelIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void LFootHeelOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void RFootToeIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void RFootToeOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void RFootHeelIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void RFootHeelOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg);*/

  // Laser:
  ros::Subscriber base_scan_sub_,rotating_scan_sub_;
  void base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );

  // Left and Right Images Seperately:
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber left_image_sub_,right_image_sub_;
  void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );

  // Combined Stereo Image:
  void stereo_cb(const sensor_msgs::ImageConstPtr& l_image,
      const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image,
      const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  const std::string stereo_in_,stereo_out_;
};

App::App(const std::string & stereo_in,
    const std::string & stereo_out):
    stereo_in_(stereo_in),
    stereo_out_(stereo_out),
    it_(node_),
    sync_(10) {
  ROS_INFO("Initializing Translator");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Clock:  
  clock_sub_ = node_.subscribe(string("/clock"), 10, &App::clock_cb,this); // previously  
  
  // Laser:
//  base_scan_sub_ = node_.subscribe(string("/scan"), 10, &App::base_scan_cb,this); // gfe
  base_scan_sub_ = node_.subscribe(string("/base_scan"), 10, &App::base_scan_cb,this); // previously
  rotating_scan_sub_ = node_.subscribe(string("/rotating_scan"), 10, &App::rotating_scan_cb,this);
  
  // Robot State:
  rstate_sub_ = node_.subscribe("true_robot_state", 10, &App::rstate_cb,this);
  
  //Foot contact sensors.
/*  LFootToeIn_cstate_sub_ = node_.subscribe("/LFootToeIn_bumper/state", 10, &App::LFootToeIn_cstate_cb,this);
  LFootToeOut_cstate_sub_ = node_.subscribe("/LFootToeOut_bumper/state", 10, &App::LFootToeOut_cstate_cb,this);
  LFootHeelIn_cstate_sub_ = node_.subscribe("/LFootHeelIn_bumper/state", 10, &App::LFootHeelIn_cstate_cb,this);
  LFootHeelOut_cstate_sub_ = node_.subscribe("/LFootHeelOut_bumper/state", 10, &App::LFootHeelOut_cstate_cb,this);
  RFootToeIn_cstate_sub_ = node_.subscribe("/RFootToeIn_bumper/state", 10, &App::RFootToeIn_cstate_cb,this);
  RFootToeOut_cstate_sub_ = node_.subscribe("/RFootToeOut_bumper/state", 10, &App::RFootToeOut_cstate_cb,this);
  RFootHeelIn_cstate_sub_ = node_.subscribe("/RFootHeelIn_bumper/state", 10, &App::RFootHeelIn_cstate_cb,this);
  RFootHeelOut_cstate_sub_ = node_.subscribe("/RFootHeelOut_bumper/state", 10, &App::RFootHeelOut_cstate_cb,this);*/
      
  // Mono-Cameras:
  left_image_sub_ = node_.subscribe(string("/left_eye/image_raw"), 10, &App::left_image_cb,this);
  right_image_sub_ = node_.subscribe(string("/right_eye/image_raw"), 10, &App::right_image_cb,this);
  //left_image_sub_ = node_.subscribe(string("/left_eye/image_rect_color"), 10, &App::left_image_cb,this);
  //right_image_sub_ = node_.subscribe(string("right_eye/image_rect_color"), 10, &App::right_image_cb,this);

  // Stereo Image:
  std::string lim_string ,lin_string,rim_string,rin_string;
  int which_image = 4;
  if (which_image==0){ // Grey:
    lim_string = stereo_in_ + "/left/image_rect";
    lin_string = stereo_in_ + "/left/camera_info";
    rim_string = stereo_in_ + "/right/image_rect";
    rin_string = stereo_in_ + "/right/camera_info";
  }else if(which_image==1){ // Color:
    lim_string = stereo_in_ + "/left/image_rect_color";
    lin_string = stereo_in_ + "/left/camera_info";
    rim_string = stereo_in_ + "/right/image_rect_color";
    rin_string = stereo_in_ + "/right/camera_info";
  }else if(which_image==2){ // Raw:
    lim_string = stereo_in_ + "/left/image_raw";
    lin_string = stereo_in_ + "/left/camera_info";
    rim_string = stereo_in_ + "/right/image_raw";
    rin_string = stereo_in_ + "/right/camera_info";
  }else if(which_image==4){ // Raw on GFE:
    // OSRF decided to despense with convention and use these channels:
    lim_string = "/left_eye/image_raw";
    lin_string = "/left_eye/camera_info";
    rim_string = "/right_eye/image_raw";
    rin_string = "/right_eye/camera_info";
  }else{
    cout << "Image choice not supported!\n";
    exit(-1); 
  }
  cout << lim_string << " is the left stereo image subscription [for stereo]\n";
  l_image_sub_.subscribe(it_, ros::names::resolve( lim_string ), 3);
  l_info_sub_.subscribe(node_, ros::names::resolve( lin_string ), 3);
  r_image_sub_.subscribe(it_, ros::names::resolve( rim_string ), 3);
  r_info_sub_.subscribe(node_, ros::names::resolve( rin_string ), 3);
  sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
  sync_.registerCallback( boost::bind(&App::stereo_cb, this, _1, _2, _3, _4) );
};

App::~App()  {
}


void App::clock_cb(const rosgraph_msgs::ClockConstPtr& msg){
  drc::utime_t utime_msg;
  utime_msg.utime = (int64_t) floor(msg->clock.toSec()  * 1E6);
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg);
}

int stereo_counter=0;
void App::stereo_cb(const sensor_msgs::ImageConstPtr& l_image,
    const sensor_msgs::CameraInfoConstPtr& l_cam_info,
    const sensor_msgs::ImageConstPtr& r_image,
    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  ros::Time current_time = l_image->header.stamp;
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toSec()  * 1E6);

  stereo_counter++;
  if (stereo_counter%10 ==0){
    std::cout << stereo_counter << " STEREO\n";
  }

  /*
  namespace enc = sensor_msgs::image_encodings;
  cv_bridge::CvImageConstPtr left_ptr;
  cv_bridge::CvImageConstPtr right_ptr;
  int isize = l_image->width*l_image->height;

  //cout << l_image->width << " w | h " << l_image->height << "\n";

  bot_core::image_t stereo;
  stereo.utime =current_utime;
  stereo.width =l_image->width;
  stereo.height =2*l_image->height;
  stereo.nmetadata =0;

  if (l_image->encoding.compare("bgr8") == 0){
    // Need to convert image to OpenCV to invert R and B:
    // OpenCV: BGR | LCM: RGB
    int n_colors = 3; // 1 is grey, 3 is rgb
    left_ptr =  cv_bridge::toCvShare(l_image, enc::BGR8);//MONO8);
    right_ptr =  cv_bridge::toCvShare(r_image, enc::BGR8);//MONO8);
    cv::Mat l_img;
    cvtColor( left_ptr->image, l_img, CV_BGR2RGB);
    cv::Mat r_img;
    cvtColor( right_ptr->image, r_img, CV_BGR2RGB);

    // Write to file:
    //cv::imwrite("left.png", left_ptr->image);
    //cv::imwrite("right.png", right_ptr->image);

    stereo.row_stride=n_colors*l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    stereo.size =n_colors *2*isize;
    copy(l_img.data, l_img.data + n_colors*isize, stereo_data);
    copy(r_img.data, r_img.data + n_colors*isize, stereo_data  + (n_colors * isize));
    stereo.data.assign(stereo_data, stereo_data + ( n_colors*2*isize));
  }else if (l_image->encoding.compare("mono8") == 0){
    stereo.row_stride=l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_GRAY;
    stereo.size =2*isize;
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (l_image->width*l_image->height));
    stereo.data.assign(stereo_data, stereo_data + ( 2*isize));
  }else if (l_image->encoding.compare("bayer_bggr8") == 0){
    stereo.row_stride=l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_BAYER_BGGR;
    stereo.size =2*isize; // I think its the same size as a typical grayscale...
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (l_image->width*l_image->height));
    stereo.data.assign(stereo_data, stereo_data + ( 2*isize));
  }else{
    cout << l_image->encoding << " image encoded not supported, not publishing\n";
    cout << stereo_out_ << "\n";
    return;
  }

  lcm_publish_.publish(stereo_out_.c_str(), &stereo);
  */

  stereo_counter++;
}


// typical contents: 640 x 480 images (out of date)
// points: xyz
// channels: rgb u v
int l_counter =0;
void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  l_counter++;
  if (l_counter%30 ==0){
    std::cout << l_counter << " left image\n";
  }  
  send_image(msg, "CAMERALEFT");
}
int r_counter =0;
void App::right_image_cb(const sensor_msgs::ImageConstPtr& msg){
  r_counter++;
  if (r_counter%30 ==0){
    std::cout << r_counter << " right image\n";
  }  
  send_image(msg, "CAMERARIGHT");
}

// TODO: jpeg compression:
//       look into openni_utils/openni_ros2rgb for code
// TODO: pre-allocate image_data and retain for speed - when size is known
void App::send_image(const sensor_msgs::ImageConstPtr& msg,string channel ){
  ros::Time current_time = msg->header.stamp;
  int64_t current_utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  /*cout << msg->width << " " << msg->height << " | "
       << msg->encoding << " is encoding | "
       << current_utime << " | "<< channel << "\n";*/

  int n_colors=3;
  int isize = msg->width*msg->height;

  bot_core::image_t lcm_img;
  lcm_img.utime =current_utime;
  lcm_img.width =msg->width;
  lcm_img.height =msg->height;
  lcm_img.nmetadata =0;
  lcm_img.row_stride=n_colors*msg->width;
  lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
  lcm_img.size =n_colors*isize;
  copy(msg->data.begin(), msg->data.end(), singleimage_data);
  lcm_img.data.assign(singleimage_data, singleimage_data + ( n_colors*2*isize));

  lcm_publish_.publish(channel.c_str(), &lcm_img);
  /*
  namespace enc = sensor_msgs::image_encodings;
  cv_bridge::CvImageConstPtr left_ptr;
  cv_bridge::CvImageConstPtr right_ptr;
  int isize = l_image->width*l_image->height;

  //cout << l_image->width << " w | h " << l_image->height << "\n";

  bot_core::image_t stereo;
  stereo.utime =current_utime;
  stereo.width =l_image->width;
  stereo.height =2*l_image->height;
  stereo.nmetadata =0;

  if (l_image->encoding.compare("bgr8") == 0){
    // Need to convert image to OpenCV to invert R and B:
    // OpenCV: BGR | LCM: RGB
    int n_colors = 3; // 1 is grey, 3 is rgb
    left_ptr =  cv_bridge::toCvShare(l_image, enc::BGR8);//MONO8);
    right_ptr =  cv_bridge::toCvShare(r_image, enc::BGR8);//MONO8);
    cv::Mat l_img;
    cvtColor( left_ptr->image, l_img, CV_BGR2RGB);
    cv::Mat r_img;
    cvtColor( right_ptr->image, r_img, CV_BGR2RGB);

    // Write to file:
    //cv::imwrite("left.png", left_ptr->image);
    //cv::imwrite("right.png", right_ptr->image);

    stereo.row_stride=n_colors*l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    stereo.size =n_colors *2*isize;
    copy(l_img.data, l_img.data + n_colors*isize, stereo_data);
    copy(r_img.data, r_img.data + n_colors*isize, stereo_data  + (n_colors * isize));
    stereo.data.assign(stereo_data, stereo_data + ( n_colors*2*isize));
  }else if (l_image->encoding.compare("mono8") == 0){
    stereo.row_stride=l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_GRAY;
    stereo.size =2*isize;
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (l_image->width*l_image->height));
    stereo.data.assign(stereo_data, stereo_data + ( 2*isize));
  }else if (l_image->encoding.compare("bayer_bggr8") == 0){
    stereo.row_stride=l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_BAYER_BGGR;
    stereo.size =2*isize; // I think its the same size as a typical grayscale...
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (l_image->width*l_image->height));
    stereo.data.assign(stereo_data, stereo_data + ( 2*isize));
  }else{
    cout << l_image->encoding << " image encoded not supported, not publishing\n";
    cout << stereo_out_ << "\n";
    return;
  }


  */



/*  image_data = new uint8_t [640*480*3];
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
  delete[] image_data;*/
}


void App::rstate_cb(const atlas_gazebo_msgs::RobotStateConstPtr& msg){

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
robot_state_msg.contacts.num_contacts =8;
robot_state_msg.contacts.id.push_back("LFootToeIn");
robot_state_msg.contacts.id.push_back("LFootToeOut");
robot_state_msg.contacts.id.push_back("LFootHeelIn");
robot_state_msg.contacts.id.push_back("LFootHeelOut");
robot_state_msg.contacts.id.push_back("RFootToeIn");
robot_state_msg.contacts.id.push_back("RFootToeOut");
robot_state_msg.contacts.id.push_back("RFootHeelIn");
robot_state_msg.contacts.id.push_back("RFootHeelOut");
for (int i=0; i< robot_state_msg.contacts.num_contacts; i++){
    robot_state_msg.contacts.inContact.push_back(0);
    drc::vector_3d_t f_zero;
    f_zero.x = 0;f_zero.y = 0;f_zero.z = 0;
    robot_state_msg.contacts.contactForce.push_back(f_zero);
}

  lcm_publish_.publish("TRUE_ROBOT_STATE", &robot_state_msg);
}


int base_scan_counter=0;
void App::base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  base_scan_counter++;
  if (base_scan_counter%30 ==0){
    std::cout << base_scan_counter << " base scan\n";
  }  
  send_lidar(msg, "BASE_SCAN");
}

void App::rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "ROTATING_SCAN");
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

  lcm_publish_.publish(channel.c_str(), &scan_out);
  //  bot_core_planar_lidar_t_publish(lcm_publish_,channel.c_str(),&scan_out);
  if (VERBOSE) cout << channel << ": "<< scan_out.utime << "\n";
}

//==================================================================================================
/*
void App::LFootToeIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){
  gazebo_msgs::ContactsState contact_states = *msg;
  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("LFootToeIn");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("LFootToeIn");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("LFOOT_TOE_IN_CONTACT_STATE", &lcm_msg);
}

void App::LFootToeOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){

  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("LFootToeOut");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("LFootToeOut");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("LFOOT_TOE_OUT_CONTACT_STATE", &lcm_msg);
}


void App::LFootHeelIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){

  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("LFootHeelIn");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("LFootHeelIn");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("LFOOT_HEEL_IN_CONTACT_STATE", &lcm_msg);
}

void App::LFootHeelOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){

  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("LFootHeelOut");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("LFootHeelOut");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("LFOOT_HEEL_OUT_CONTACT_STATE", &lcm_msg);
}

void App::RFootToeIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){

  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("RFootToeIn");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("RFootToeIn");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("RFOOT_TOE_IN_CONTACT_STATE", &lcm_msg);
}

void App::RFootToeOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){

  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("RFootToeOut");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("RFootToeOut");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("RFOOT_TOE_OUT_CONTACT_STATE", &lcm_msg);
}


void App::RFootHeelIn_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){

  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("RFootHeelIn");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("RFootHeelIn");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("RFOOT_HEEL_IN_CONTACT_STATE", &lcm_msg);
}

void App::RFootHeelOut_cstate_cb(const gazebo_msgs::ContactsState::ConstPtr& msg){

  drc::contact_state_stamped_t lcm_msg;
  lcm_msg.utime = msg->header.stamp.toNSec()/1000; // from nsec to usec
  lcm_msg.robot_name = "atlas"; // this cb is active only for atlas
  lcm_msg.num_contacts = 1;

  if(msg->states.empty()) {
      bool inContact = false;
      
       drc::vector_3d_t  f;
       f.x = 0;
       f.y = 0;
       f.z = 0;
       lcm_msg.id.push_back("RFootHeelOut");
       lcm_msg.inContact.push_back(inContact);
       lcm_msg.contactForce.push_back(f);
  }
  else{

   bool inContact = true;
    drc::vector_3d_t  f;
     f.x = 0;  f.y = 0;   f.z = 0;
     for (unsigned int k=0; k < msg->states.size(); k++) // for multiple collisions (usually one)
     {
         geometry_msgs::Wrench total_wrench;
         total_wrench = msg->states[k].total_wrench;
         
         f.x = f.x + total_wrench.force.x;
         f.y = f.y + total_wrench.force.y;
         f.z = f.z + total_wrench.force.z;

          //total_wrench.torque.x; 
          //total_wrench.torque.y;
          //total_wrench.torque.z; 
     }

    lcm_msg.id.push_back("RFootHeelOut");
    lcm_msg.inContact.push_back(inContact);
    lcm_msg.contactForce.push_back(f);
  }
  lcm_publish_.publish("RFOOT_HEEL_OUT_CONTACT_STATE", &lcm_msg);
}
*/
//==================================================================================================


int main(int argc, char **argv){
  std::cout << "ros2lcm_translator_spinning launched\n";
  ros::init(argc, argv, "ros2lcm_translator_spinning");

  App *app = new App("camera", "CAMERA");

  /*
  std::cout << "Arguments: ros2lcm_translator_combined ROS_stereo_channel LCM_stereo_channel\n";
  std::cout << "ros2lcm_translator_combined camera WIDE_STEREO_IMAGE\n";
  std::cout << "    program: " << argv[ 0 ] << "\n";
  std::cout << "ROS channel: " << argv[ 1 ] << " [Input]\n";
  std::cout << "LCM channel: " << argv[ 2 ] << " [Output]\n";
  App *app = new App(argv[ 1 ], argv[ 2 ]);*/
  ros::spin();
  return 0;
}
