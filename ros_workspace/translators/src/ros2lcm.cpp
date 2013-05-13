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
#include <cv_bridge/cv_bridge.h>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <atlas_msgs/ForceTorqueSensors.h>

// deprecated:
//#include <atlas_gazebo_msgs/RobotState.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <ros/callback_queue.h>

#include <ConciseArgs>


#define VERBOSE false
using namespace std;


int width =800;//1024; // hardcoded
int height =800;//544; // hardcoded
uint8_t* stereo_data = new uint8_t [2*3* width*height]; // 2 color scale images stacked
uint8_t* singleimage_data = new uint8_t [2*2* width*height]; // 1 color scale image

class App{
public:
  App(const std::string & stereo_in, const std::string & stereo_out, ros::NodeHandle node_, string mode_, 
      bool control_output_, bool perception_output_, bool send_ground_truth_, bool send_hand_cameras_);
  ~App();

private:
  string mode_;
  bool control_output_; // publish control msgs to LCM
  bool perception_output_; // publish control msgs to LCM
  bool send_ground_truth_; // listen for and publish ground truth inside TRUE_ROBOT_STATE 
  bool send_hand_cameras_; // listen for and publish the hand cameras
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Clock:
  ros::Subscriber clock_sub_;
  void clock_cb(const rosgraph_msgs::ClockConstPtr& msg);
  
  // All of this data is mashed down into one LCM message - Robot State ////
//  ros::Subscriber rstate_sub_;
//  void rstate_cb(const atlas_gazebo_msgs::RobotStateConstPtr& msg);
  sensor_msgs::JointState l_hand_joint_states_, r_hand_joint_states_, robot_joint_states_, head_joint_states_;  
  ros::Subscriber  joint_states_sub_, l_hand_joint_states_sub_, r_hand_joint_states_sub_, head_joint_states_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void l_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void r_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  void head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  void appendJointStates(drc::robot_state_t& msg_out, sensor_msgs::JointState msg_in); 
  void appendLimbSensor(drc::robot_state_t& msg_out , atlas_msgs::ForceTorqueSensors msg_in);
  void publishRobotState(int64_t utime_in);
  bool init_recd_[2]; // have recived gt [0], robot joints [1]

  ros::Subscriber end_effector_sensors_sub_;  
  void end_effector_sensors_cb(const atlas_msgs::ForceTorqueSensorsConstPtr& msg);  
  atlas_msgs::ForceTorqueSensors end_effector_sensors_;
  
  ros::Subscriber ground_truth_odom_sub_;  
  void ground_truth_odom_cb(const nav_msgs::OdometryConstPtr& msg);  
  nav_msgs::Odometry ground_truth_odom_;
  //////////////////////////////////////////////////////////////////////////
  
  // Imu:
  ros::Subscriber torso_imu_sub_,head_imu_sub_;
  void torso_imu_cb(const sensor_msgs::ImuConstPtr& msg);
  void head_imu_cb(const sensor_msgs::ImuConstPtr& msg);
  void send_imu(const sensor_msgs::ImuConstPtr& msg,string channel );
  void send_imu_as_pose(const sensor_msgs::ImuConstPtr& msg,string channel );

  // Laser:
  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  ros::Subscriber rotating_scan_sub_;
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  // porterbot:
  ros::Subscriber scan_left_sub_,scan_right_sub_; // porterbot
  void scan_left_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void scan_right_cb(const sensor_msgs::LaserScanConstPtr& msg);

  // Left and Right Images Seperately:
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber left_image_sub_,right_image_sub_;
  void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );
  
  // Combined Stereo Image:
  void stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info, std::string camera_out);
  image_transport::ImageTransport it_;
  
  void head_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  
  void l_hand__stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter l_hand_l_image_sub_, l_hand_r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_hand_l_info_sub_, l_hand_r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> l_hand_sync_;
  
  void r_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter r_hand_l_image_sub_, r_hand_r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> r_hand_l_info_sub_, r_hand_r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> r_hand_sync_;
  
  const std::string stereo_in_,stereo_out_;
};

App::App(const std::string & stereo_in,
    const std::string & stereo_out,
    ros::NodeHandle node_, string mode_, 
    bool control_output_, bool perception_output_, bool send_ground_truth_, bool send_hand_cameras_) :
    stereo_in_(stereo_in), stereo_out_(stereo_out),
    node_(node_), it_(node_), sync_(10), l_hand_sync_(10), r_hand_sync_(10),
    mode_(mode_), control_output_(control_output_), 
    perception_output_(perception_output_), send_ground_truth_(send_ground_truth_), send_hand_cameras_(send_hand_cameras_){
  ROS_INFO("Initializing Translator");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Clock:  
  if (control_output_){
    clock_sub_ = node_.subscribe(string("/clock"), 10, &App::clock_cb,this);

    // IMU:
    torso_imu_sub_ = node_.subscribe(string("/atlas/imu"), 10, &App::torso_imu_cb,this);
    head_imu_sub_ = node_.subscribe(string("/multisense_sl/imu"), 10, &App::head_imu_cb,this);

    // Robot State:
    //rstate_sub_ = node_.subscribe("true_robot_state", 1000, &App::rstate_cb,this, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    //rstate_sub_ = node_.subscribe("true_robot_state", 10, &App::rstate_cb,this);
    joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 1000, &App::joint_states_cb,this, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    head_joint_states_sub_ = node_.subscribe(string("/multisense_sl/joint_states"), 1000, &App::head_joint_states_cb,this, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    l_hand_joint_states_sub_ = node_.subscribe(string("/sandia_hands/l_hand/joint_states"), 1000, &App::l_hand_joint_states_cb,this, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    r_hand_joint_states_sub_ = node_.subscribe(string("/sandia_hands/r_hand/joint_states"), 1000, &App::r_hand_joint_states_cb,this, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    end_effector_sensors_sub_ = node_.subscribe(string("/atlas/force_torque_sensors"), 10, &App::end_effector_sensors_cb,this);

    init_recd_[0]=false; // had ground_truth_odom been received?
    init_recd_[1]=false; // have joint angles been received?
    
    ///////////////////////////// Ground Truth Odom ///////////////////////////////////////
    // initialize with known (but ridiculous) state - to expose any GT data feeding through
    ground_truth_odom_.pose.pose.position.x =0.;
    ground_truth_odom_.pose.pose.position.y =0.;
    ground_truth_odom_.pose.pose.position.z =0.;
    ground_truth_odom_.pose.pose.orientation.w =0.; // upside down (to make it obvious)
    ground_truth_odom_.pose.pose.orientation.x =1.;
    ground_truth_odom_.pose.pose.orientation.y =0.;
    ground_truth_odom_.pose.pose.orientation.z =0.;
    ground_truth_odom_.twist.twist.linear.x = std::numeric_limits<int>::min();
    ground_truth_odom_.twist.twist.linear.y = std::numeric_limits<int>::min();
    ground_truth_odom_.twist.twist.linear.z = std::numeric_limits<int>::min();
    ground_truth_odom_.twist.twist.angular.x = std::numeric_limits<int>::min();
    ground_truth_odom_.twist.twist.angular.y = std::numeric_limits<int>::min();
    ground_truth_odom_.twist.twist.angular.z = std::numeric_limits<int>::min();
    
    if (send_ground_truth_){
      ground_truth_odom_sub_ = node_.subscribe(string("/ground_truth_odom"), 1000, &App::ground_truth_odom_cb,this, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    }else{
      init_recd_[0]=true; // NB: needed to fool this program into believing it has got GTO when we aren't going to provide it
    }
    
  }
  
  if (perception_output_){
    // Laser:
    rotating_scan_sub_ = node_.subscribe(string("/multisense_sl/laser/scan"), 10, &App::rotating_scan_cb,this);
    // Porterbot
    //scan_left_sub_ = node_.subscribe(string("/scan_left"), 10, &App::scan_left_cb,this);
    //scan_right_sub_ = node_.subscribe(string("/scan_right"), 10, &App::scan_right_cb,this);
  
    bool send_cameras =true;
    if (send_cameras){
      
      // Stereo Image:
      std::string lim_string ,lin_string,rim_string,rin_string;
      int which_image = 0;
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
      }else if(which_image==4){ // Raw on GFE: (is RGB)
	lim_string = stereo_in_ + "/left/image_raw";
	lin_string = stereo_in_ + "/left/camera_info";
	rim_string = stereo_in_ + "/right/image_raw";
	rin_string = stereo_in_ + "/right/camera_info";
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
      sync_.registerCallback( boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4) );

      // Mono-Cameras:
      left_image_sub_ = node_.subscribe( string(stereo_in_ + "/left/image_raw"), 10, &App::left_image_cb,this);
      if (1==0){
	left_image_sub_ = node_.subscribe( lim_string, 10, &App::left_image_cb,this);
	right_image_sub_ = node_.subscribe(rim_string, 10, &App::right_image_cb,this);
      }

      /////////////////////////////// Hand Cameras /////////////////////////////////////
      if (send_hand_cameras_){
	l_hand_l_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/l_hand/camera/left/image_raw" ), 3);
	l_hand_l_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/l_hand/camera/left/camera_info" ), 3);
	l_hand_r_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/l_hand/camera/right/image_raw" ), 3);
	l_hand_r_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/l_hand/camera/right/camera_info"  ), 3);
	l_hand_sync_.connectInput(l_hand_l_image_sub_, l_hand_l_info_sub_, l_hand_r_image_sub_, l_hand_r_info_sub_);
	l_hand_sync_.registerCallback( boost::bind(&App::l_hand__stereo_cb, this, _1, _2, _3, _4) );        

	r_hand_l_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/r_hand/camera/left/image_raw" ), 3);
	r_hand_l_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/r_hand/camera/left/camera_info" ), 3);
	r_hand_r_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/r_hand/camera/right/image_raw" ), 3);
	r_hand_r_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/r_hand/camera/right/camera_info"  ), 3);
	r_hand_sync_.connectInput(r_hand_l_image_sub_, r_hand_l_info_sub_, r_hand_r_image_sub_, r_hand_r_info_sub_);
	r_hand_sync_.registerCallback( boost::bind(&App::r_hand_stereo_cb, this, _1, _2, _3, _4) );   
      }
    
    }
  }
};

App::~App()  {
}

void App::send_imu(const sensor_msgs::ImuConstPtr& msg,string channel ){
  drc::imu_t imu_msg;
  imu_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);  
  //imu_msg.frame_id = msg->header->frame_id;
  imu_msg.orientation[0] = msg->orientation.w; // NB: order here is wxyz
  imu_msg.orientation[1] = msg->orientation.x;
  imu_msg.orientation[2] = msg->orientation.y;
  imu_msg.orientation[3] = msg->orientation.z;
  imu_msg.angular_velocity[0] = msg->angular_velocity.x;
  imu_msg.angular_velocity[1] = msg->angular_velocity.y;
  imu_msg.angular_velocity[2] = msg->angular_velocity.z;
  imu_msg.linear_acceleration[0] = msg->linear_acceleration.x;
  imu_msg.linear_acceleration[1] = msg->linear_acceleration.y;
  imu_msg.linear_acceleration[2] = msg->linear_acceleration.z;
  for (size_t i=0;i<9;i++){
    imu_msg.orientation_covariance[i] = msg->orientation_covariance[i];
    imu_msg.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
    imu_msg.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
  }
  lcm_publish_.publish(channel, &imu_msg);
}
void App::send_imu_as_pose(const sensor_msgs::ImuConstPtr& msg,string channel ){
  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);  
  pose_msg.pos[0] = 0;
  pose_msg.pos[1] = 0;
  pose_msg.pos[2] = 0;
  pose_msg.orientation[0] = msg->orientation.w;
  pose_msg.orientation[1] = msg->orientation.x;
  pose_msg.orientation[2] = msg->orientation.y;
  pose_msg.orientation[3] = msg->orientation.z;
  lcm_publish_.publish(channel, &pose_msg);
}

void App::torso_imu_cb(const sensor_msgs::ImuConstPtr& msg){
  send_imu(msg,"TORSO_IMU");
  send_imu_as_pose(msg,"POSE_BODY_ORIENT"); // not necessary, just for easy rendering in bot_frames in viewer 
}

void App::head_imu_cb(const sensor_msgs::ImuConstPtr& msg){
  send_imu(msg,"HEAD_IMU");
  send_imu_as_pose(msg,"POSE_HEAD_ORIENT"); // not necessary, just for easy rendering in bot_frames in viewer 
}

void App::clock_cb(const rosgraph_msgs::ClockConstPtr& msg){
  drc::utime_t utime_msg;
  utime_msg.utime = (int64_t) floor(msg->clock.toNSec()/1000);
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg);
}


void App::head_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  stereo_cb(l_image,l_cam_info,r_image,r_cam_info,"CAMERA");
}

void App::l_hand__stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  stereo_cb(l_image,l_cam_info,r_image,r_cam_info,"CAMERA_LHAND");
}

void App::r_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  stereo_cb(l_image,l_cam_info,r_image,r_cam_info,"CAMERA_RHAND");
}

int stereo_counter=0;
void App::stereo_cb(const sensor_msgs::ImageConstPtr& l_image,
    const sensor_msgs::CameraInfoConstPtr& l_cam_info,
    const sensor_msgs::ImageConstPtr& r_image,
    const sensor_msgs::CameraInfoConstPtr& r_cam_info, std::string camera_out)
{
  
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  stereo_counter++;
  if (stereo_counter%30 ==0){
    std::cout << stereo_counter << " stereo images -> CAMERA \n";
  }

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
    
    //camera_out = "LHAND_CAMERA";
  }else if (l_image->encoding.compare("bayer_bggr8") == 0){
    stereo.row_stride=l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_BAYER_BGGR;
    stereo.size =2*isize; // I think its the same size as a typical grayscale...
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (l_image->width*l_image->height));
    stereo.data.assign(stereo_data, stereo_data + ( 2*isize));
  }else if (l_image->encoding.compare("rgb8") == 0){
    // Need to convert image to OpenCV to invert R and B:
    // OpenCV: BGR | LCM: RGB
    int n_colors = 3; // 1 is grey, 3 is rgb
    stereo.row_stride=n_colors*l_image->width;
    stereo.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    stereo.size =n_colors *2*isize;
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data +  n_colors*isize);
    stereo.data.assign(stereo_data, stereo_data + ( n_colors*2*isize));   
  }else{
    cout << l_image->encoding << " image encoded not supported, not publishing\n";
    cout << stereo_out_ << "\n";
    return;
  }
  lcm_publish_.publish(camera_out, &stereo);
//  lcm_publish_.publish(stereo_out_.c_str(), &stereo);
  
  // As a convenience also publish the left image:
  //send_image(l_image, "CAMERALEFT" );
}


// typical contents: 640 x 480 images (out of date)
// points: xyz
// channels: rgb u v
int l_counter =0;
void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  if (l_counter%30 ==0){
    std::cout << l_counter << " left image\n";
  }  
  l_counter++;
  send_image(msg, "CAMERALEFT");
}
int r_counter =0;
void App::right_image_cb(const sensor_msgs::ImageConstPtr& msg){
  std::cout << "got r image\n";
  std::cout << msg->encoding << " is r encdoing\n";
  
  if (r_counter%30 ==0){
    std::cout << r_counter << " right image\n";
  }  
  r_counter++;
  send_image(msg, "CAMERARIGHT");
}

// TODO: jpeg compression:
//       look into openni_utils/openni_ros2rgb for code
// TODO: pre-allocate image_data and retain for speed - when size is known
void App::send_image(const sensor_msgs::ImageConstPtr& msg,string channel ){
  int64_t current_utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
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
  lcm_img.data.assign(singleimage_data, singleimage_data + ( n_colors*isize));

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
  img.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
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


void App::end_effector_sensors_cb(const atlas_msgs::ForceTorqueSensorsConstPtr& msg){
  end_effector_sensors_ = *msg;
}

int gt_counter =0;
void App::ground_truth_odom_cb(const nav_msgs::OdometryConstPtr& msg){
  if (gt_counter%200 ==0){
    std::cout << gt_counter << " ground truth\n";
  }  
  gt_counter++;

  ground_truth_odom_ = *msg;
  init_recd_[0] =true;
}

/// Locally cache the joint states:
int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  if (js_counter%500 ==0){
    std::cout << js_counter << " joint states\n";
  }  
  js_counter++;
  
  
  robot_joint_states_ = *msg; 
  init_recd_[1] =true; 
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  publishRobotState(joint_utime);
}
void App::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  head_joint_states_= *msg;
}
void App::l_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  l_hand_joint_states_= *msg;
  
  if (mode_.compare("hands") == 0){
    int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
    ground_truth_odom_.pose.pose.orientation.w =1;
    publishRobotState(joint_utime);
  }
}
void App::r_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  r_hand_joint_states_= *msg;
}



void App::appendJointStates(drc::robot_state_t& msg_out , sensor_msgs::JointState msg_in){
  drc::joint_covariance_t j_cov;
  j_cov.variance = 0;
  for (std::vector<int>::size_type i = 0; i < msg_in.name.size(); i++)  {
    msg_out.joint_name.push_back(msg_in.name[i]);
    msg_out.joint_position.push_back(msg_in.position[i]);
    msg_out.joint_velocity.push_back(msg_in.velocity[i]);
    msg_out.measured_effort.push_back( msg_in.effort[i] );
    msg_out.joint_cov.push_back(j_cov);
  }
}

void App::appendLimbSensor(drc::robot_state_t& msg_out , atlas_msgs::ForceTorqueSensors msg_in){
  {
    drc::vector_3d_t l_foot_force;
    l_foot_force.x = msg_in.l_foot.force.x; l_foot_force.y = msg_in.l_foot.force.y; l_foot_force.z = msg_in.l_foot.force.z;
    drc::vector_3d_t l_foot_torque;
    l_foot_torque.x = msg_in.l_foot.torque.x; l_foot_torque.y = msg_in.l_foot.torque.y; l_foot_torque.z = msg_in.l_foot.torque.z;
    msg_out.contacts.id.push_back("l_foot");
    msg_out.contacts.contact_force.push_back(l_foot_force);
    msg_out.contacts.contact_torque.push_back(l_foot_torque);  
  }
  
  {
    drc::vector_3d_t r_foot_force;
    r_foot_force.x = msg_in.r_foot.force.x; r_foot_force.y = msg_in.r_foot.force.y; r_foot_force.z = msg_in.r_foot.force.z;
    drc::vector_3d_t r_foot_torque;
    r_foot_torque.x = msg_in.r_foot.torque.x; r_foot_torque.y = msg_in.r_foot.torque.y; r_foot_torque.z = msg_in.r_foot.torque.z;
    msg_out.contacts.id.push_back("r_foot");
    msg_out.contacts.contact_force.push_back(r_foot_force);
    msg_out.contacts.contact_torque.push_back(r_foot_torque);  
  }
  
  {
    drc::vector_3d_t l_hand_force;
    l_hand_force.x = msg_in.l_hand.force.x; l_hand_force.y = msg_in.l_hand.force.y; l_hand_force.z = msg_in.l_hand.force.z;
    drc::vector_3d_t l_hand_torque;
    l_hand_torque.x = msg_in.l_hand.torque.x; l_hand_torque.y = msg_in.l_hand.torque.y; l_hand_torque.z = msg_in.l_hand.torque.z;
    msg_out.contacts.id.push_back("l_hand");
    msg_out.contacts.contact_force.push_back(l_hand_force);
    msg_out.contacts.contact_torque.push_back(l_hand_torque);  
  }
    
  {
    drc::vector_3d_t r_hand_force;
    r_hand_force.x = msg_in.r_hand.force.x; r_hand_force.y = msg_in.r_hand.force.y; r_hand_force.z = msg_in.r_hand.force.z;
    drc::vector_3d_t r_hand_torque;
    r_hand_torque.x = msg_in.r_hand.torque.x; r_hand_torque.y = msg_in.r_hand.torque.y; r_hand_torque.z = msg_in.r_hand.torque.z;
    msg_out.contacts.id.push_back("r_hand");
    msg_out.contacts.contact_force.push_back(r_hand_force);
    msg_out.contacts.contact_torque.push_back(r_hand_torque);  
  }    
  
}

void App::publishRobotState(int64_t utime_in){
 // If haven't got a ground_truth_odom_, exit:
 if(mode_.compare("hands") != 0){
  if(!init_recd_[0])
    return;
  if(!init_recd_[1])
    return;
 }
  
  
  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = utime_in;
  robot_state_msg.robot_name = "atlas";
  
  // Pelvis Pose:
  robot_state_msg.origin_position.translation.x = ground_truth_odom_.pose.pose.position.x;
  robot_state_msg.origin_position.translation.y = ground_truth_odom_.pose.pose.position.y;
  robot_state_msg.origin_position.translation.z = ground_truth_odom_.pose.pose.position.z;
  robot_state_msg.origin_position.rotation.w = ground_truth_odom_.pose.pose.orientation.w;
  robot_state_msg.origin_position.rotation.x = ground_truth_odom_.pose.pose.orientation.x;
  robot_state_msg.origin_position.rotation.y = ground_truth_odom_.pose.pose.orientation.y;
  robot_state_msg.origin_position.rotation.z = ground_truth_odom_.pose.pose.orientation.z;

  robot_state_msg.origin_twist.linear_velocity.x =ground_truth_odom_.twist.twist.linear.x;
  robot_state_msg.origin_twist.linear_velocity.y =ground_truth_odom_.twist.twist.linear.y;
  robot_state_msg.origin_twist.linear_velocity.z =ground_truth_odom_.twist.twist.linear.z;
  robot_state_msg.origin_twist.angular_velocity.x =ground_truth_odom_.twist.twist.angular.x;
  robot_state_msg.origin_twist.angular_velocity.y =ground_truth_odom_.twist.twist.angular.y;
  robot_state_msg.origin_twist.angular_velocity.z =ground_truth_odom_.twist.twist.angular.z;
  for(int i = 0; i < 6; i++)  {
    for(int j = 0; j < 6; j++) {
      robot_state_msg.origin_cov.position_cov[i][j] = 0;
      robot_state_msg.origin_cov.twist_cov[i][j] = 0;
    }
  }

  // Joint States:
  appendJointStates(robot_state_msg, robot_joint_states_);
  appendJointStates(robot_state_msg, head_joint_states_);
  appendJointStates(robot_state_msg, l_hand_joint_states_);
  appendJointStates(robot_state_msg, r_hand_joint_states_);
  robot_state_msg.num_joints = robot_state_msg.joint_name.size();
  
  // Limb Sensor states
  appendLimbSensor(robot_state_msg, end_effector_sensors_);
  robot_state_msg.contacts.num_contacts = robot_state_msg.contacts.contact_torque.size();
    
  lcm_publish_.publish("TRUE_ROBOT_STATE", &robot_state_msg);    
}



/*
void App::rstate_cb(const atlas_gazebo_msgs::RobotStateConstPtr& msg){
  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
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
*/


int scan_counter=0;
void App::rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  if (scan_counter%80 ==0){
    std::cout << scan_counter << " /multisense_sl/laser/scan -> SCAN\n";
  }  
  scan_counter++;
  send_lidar(msg, "SCAN");
}
void App::scan_left_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "SCAN_LEFT");
}
void App::scan_right_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "SCAN_RIGHT");
}

void App::send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities; // currently all identical
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;

  lcm_publish_.publish(channel.c_str(), &scan_out);
  if (VERBOSE) cout << channel << ": "<< scan_out.utime << "\n";
}

int main(int argc, char **argv){
  ConciseArgs parser(argc, argv, "ros2lcm");
  string mode = "robot";
  bool control_output = false;
  bool perception_output = false;  
  bool send_ground_truth = false;  
  bool send_hand_cameras = false;
  parser.add(mode, "m", "mode", "Mode: robot, hands");
  parser.add(control_output, "c", "control_output", "Publish control message");
  parser.add(perception_output, "p", "perception_output", "Publish perception messages");
  parser.add(send_ground_truth, "g", "send_ground_truth", "Listen for and include GT odom");
  parser.add(send_hand_cameras, "l", "send_hand_cameras", "Send cameras in hand/limbs");
  parser.parse();
  cout << "Publish Mode: " << mode << "\n";   
  cout << "Publish Control Messages: " << control_output << "\n";   
  cout << "Publish Perception Messages: " << perception_output << "\n";   
  cout << "Listen for and publish ground_truth_odom: " << send_ground_truth << " [control mode only]\n";   
  cout << "Publish Hand Camera Messages: " << send_hand_cameras << "\n";   
  
  if (control_output && perception_output){
    cout << "must run to translators\ncontrol_output and perception_output cannot both be true\n";
    return -1;
  }else if(control_output){
    ros::init(argc, argv, "ros2lcm_control");
  }else if(perception_output){
    ros::init(argc, argv, "ros2lcm_perception");
  }else{
    cout << "Please choose a mode: control_output or perception_output (but not both)\n";
    return -1;
  }
  
  ros::CallbackQueue local_callback_queue;
  ros::NodeHandle nh;
  nh.setCallbackQueue(&local_callback_queue);
  
  App *app = new App( "multisense_sl/camera", "CAMERA", nh, mode, control_output, perception_output, send_ground_truth, send_hand_cameras);
  std::cout << "ros2lcm translator ready\n";
  while (ros::ok()){
    local_callback_queue.callAvailable(ros::WallDuration(0.01));
  }  
  return 0;
}
