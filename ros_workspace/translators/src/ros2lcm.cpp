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
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <atlas_msgs/VRCScore.h>
#include <std_msgs/Float64.h>


#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <ros/callback_queue.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression


#include <ConciseArgs>

#define DO_TIMING_PROFILE true

#define VERBOSE false
using namespace std;


int width =800;//1024; // hardcoded
int height =800;//544; // hardcoded
uint8_t* stereo_data = new uint8_t [2*3* width*height]; // 2 color scale images stacked
uint8_t* singleimage_data = new uint8_t [2*2* width*height]; // 1 color scale image

class App{
public:
  App(ros::NodeHandle node_, string mode_, 
      bool control_output_, bool send_ground_truth_, bool send_head_cameras_, bool send_hand_cameras_);
  ~App();

private:
  string mode_;
  bool control_output_; // publish control msgs to LCM
  bool send_ground_truth_; // listen for and publish ground truth inside TRUE_ROBOT_STATE 
  bool send_head_cameras_;  // listen for and publish the cameras
  bool send_hand_cameras_; // listen for and publish the hand cameras (the above must be true also!)
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Clock:
  ros::Subscriber clock_sub_;
  void clock_cb(const rosgraph_msgs::ClockConstPtr& msg);
  // VRC Score:
  ros::Subscriber vrc_score_sub_;
  void vrc_score_cb(const atlas_msgs::VRCScoreConstPtr& msg);
  
  // All of this data is mashed down into one LCM message - Robot State ////
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

  // Rate messages:
  double hand_stereo_publish_fps_, head_stereo_publish_fps_;
  ros::Subscriber head_fps_sub_,hand_fps_sub_;
  void head_fps_cb(const std_msgs::Float64ConstPtr& msg);
  void hand_fps_cb(const std_msgs::Float64ConstPtr& msg);  
  
  
  // Left Image Seperately:
  int64_t head_left_last_utime_;
  ros::Subscriber left_image_sub_;  
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );
  
  // Combined Stereo Image:
  bot_core::image_t stereo_msg_out_;  
//  void publishStereo(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
//      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info, std::string camera_out);
  image_transport::ImageTransport it_;
  
  
  ///////////////////////////////////////////////////////////////////////////////
  int64_t head_stereo_last_utime_;
  void head_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  int64_t l_hand_stereo_last_utime_;
  
  /*
  void l_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter l_hand_l_image_sub_, l_hand_r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_hand_l_info_sub_, l_hand_r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> l_hand_sync_;
  
  int64_t r_hand_stereo_last_utime_;
  void r_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter r_hand_l_image_sub_, r_hand_r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> r_hand_l_info_sub_, r_hand_r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> r_hand_sync_;
  */
  
  
  // Image Compression
  image_io_utils*  imgutils_;
};

App::App(ros::NodeHandle node_, string mode_, 
    bool control_output_, bool send_ground_truth_, bool send_head_cameras_, bool send_hand_cameras_) :
    node_(node_), it_(node_), mode_(mode_),
    sync_(10), //l_hand_sync_(10), r_hand_sync_(10),
    control_output_(control_output_), 
    send_ground_truth_(send_ground_truth_), 
    send_head_cameras_(send_head_cameras_), send_hand_cameras_(send_hand_cameras_){
  ROS_INFO("Initializing Translator");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  imgutils_ = new image_io_utils( lcm_publish_.getUnderlyingLCM(), width, height );
  

  // Clock:  
  if (control_output_){
    // in the joint states pub now:
    //   clock_sub_ = node_.subscribe(string("/clock"), 10, &App::clock_cb,this);
    
    vrc_score_sub_ = node_.subscribe(string("/vrc_score"), 100, &App::vrc_score_cb,this);
    

    // IMU:
    torso_imu_sub_ = node_.subscribe(string("/atlas/imu"), 100, &App::torso_imu_cb,this);

    // Robot State:
    joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 100, &App::joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    head_joint_states_sub_ = node_.subscribe(string("/multisense_sl/joint_states"), 100, &App::head_joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    l_hand_joint_states_sub_ = node_.subscribe(string("/sandia_hands/l_hand/joint_states"), 100, &App::l_hand_joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    r_hand_joint_states_sub_ = node_.subscribe(string("/sandia_hands/r_hand/joint_states"), 100, &App::r_hand_joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    end_effector_sensors_sub_ = node_.subscribe(string("/atlas/force_torque_sensors"), 100, &App::end_effector_sensors_cb,this);

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
      ground_truth_odom_sub_ = node_.subscribe(string("/ground_truth_odom"), 100, &App::ground_truth_odom_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
    }else{
      init_recd_[0]=true; // NB: needed to fool this program into believing it has got GTO when we aren't going to provide it
    }

  }
  
  // Cameras have been moved!
  
};

App::~App()  {
}

void App::vrc_score_cb(const atlas_msgs::VRCScoreConstPtr& msg){
  //std::cout << "got vrc score\n";  
  drc::score_t score;
  score.wall_time=(int64_t) floor(msg->wall_time.toNSec()/1000);  
  score.sim_time=(int64_t) floor(msg->sim_time.toNSec()/1000);  
  score.wall_time_elapsed=(int64_t) floor(msg->wall_time_elapsed.toNSec()/1000);  
  score.sim_time_elapsed=(int64_t) floor(msg->sim_time_elapsed.toNSec()/1000);  
  score.completion_score = msg->completion_score;
  score.falls = msg->falls;
  score.task_type = msg->task_type;
  lcm_publish_.publish("VRC_SCORE", &score);

  if (!msg->message.empty() ){
    // any messages from gazebo come as occasional strings:
    drc::system_status_t m;
    m.utime = (int64_t) floor(msg->sim_time.toNSec()/1000);
    m.system = drc::system_status_t::MESSAGING;
    m.importance = drc::system_status_t::VERY_IMPORTANT;
    m.frequency = drc::system_status_t::LOW_FREQUENCY;

    std::stringstream ss;
    ss << "VRC Score: " 
       <<  msg->wall_time_elapsed.toSec() << " Seconds Elapsed | "
       <<  msg->completion_score << " Completion | "
       <<  msg->falls << " Falls | "
       <<  (int) msg->task_type << " Task";
    m.value = ss.str();
    std::cout << ss.str() << " \n";  
    lcm_publish_.publish("SYSTEM_STATUS", &m); // for simplicity stick this out
    
    m.value = string( "VRC Score: " ) + msg->message;
    lcm_publish_.publish("SYSTEM_STATUS", &m);
    lcm_publish_.publish("VRC_SCORE_MESSAGE", &m); // for conevenince
  }
}



void App::head_fps_cb(const std_msgs::Float64ConstPtr& msg){
  head_stereo_publish_fps_ = double( msg->data);
  std::cout << "set the head stereo fps to : " << head_stereo_publish_fps_ << "\n";
}

void App::hand_fps_cb(const std_msgs::Float64ConstPtr& msg){
  hand_stereo_publish_fps_ = double( msg->data);
  std::cout << "set the hand stereo fps to : " << hand_stereo_publish_fps_ << "\n";  
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
  //send_imu_as_pose(msg,"POSE_BODY_ORIENT"); // not necessary, just for easy rendering in bot_frames in viewer 
}

void App::head_imu_cb(const sensor_msgs::ImuConstPtr& msg){
  send_imu(msg,"HEAD_IMU");
  //send_imu_as_pose(msg,"POSE_HEAD_ORIENT"); // not necessary, just for easy rendering in bot_frames in viewer 
}

void App::clock_cb(const rosgraph_msgs::ClockConstPtr& msg){
  drc::utime_t utime_msg;
  utime_msg.utime = (int64_t) floor(msg->clock.toNSec()/1000);
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg);
}


int stereo_counter=0;
void App::head_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info){
  std::cout << "head_stereo_cb is currently disabled mfallon\n";
  
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  //if ( float(current_utime - head_stereo_last_utime_) > float(1E6/head_stereo_publish_fps_ ) ){
    //std::cout << "send head\n";
    head_stereo_last_utime_ = current_utime;
    //publishStereo(l_image,l_cam_info,r_image,r_cam_info,"CAMERA");
    
    if (stereo_counter%30 ==0){
      std::cout << "HCAM " << stereo_counter << "\n";
    }
    stereo_counter++;
  
  //}else{
    //std::cout << "dont send head\n";
  //}  
}


/*

int lhand_stereo_counter=0;
void App::l_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  if ( float(current_utime -l_hand_stereo_last_utime_) > float(1E6/hand_stereo_publish_fps_ ) ){
    //std::cout << "send l hand\n";
    l_hand_stereo_last_utime_ = current_utime;
    //publishStereo(l_image,l_cam_info,r_image,r_cam_info,"CAMERA_LHAND");
    
    if (lhand_stereo_counter%30 ==0){
      std::cout << "LCAM " << lhand_stereo_counter << "\n";
    }
    lhand_stereo_counter++;

  }else{
    //std::cout << "dont send hand l\n";
  }
}

int rhand_stereo_counter=0;
void App::r_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  if ( float(current_utime - r_hand_stereo_last_utime_) > float(1E6/hand_stereo_publish_fps_ ) ){
    //std::cout << "send r hand\n";
    r_hand_stereo_last_utime_ = current_utime;
    //publishStereo(l_image,l_cam_info,r_image,r_cam_info,"CAMERA_RHAND");
    
    if (rhand_stereo_counter%30 ==0){
      std::cout << "RCAM " << rhand_stereo_counter << "\n";
    }    
    rhand_stereo_counter++;
  }else{
    //std::cout << "dont send r hand\n";
  }
}

*/

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

// display_tic_toc: a helper function which accepts a set of 
// timestamps and displays the elapsed time between them as 
// a fraction and time used [for profiling]
void display_tic_toc(std::vector<int64_t> &tic_toc,const std::string &fun_name){
  int tic_toc_size = tic_toc.size();
  double percent_tic_toc_last = 0;
  double dtime = ((double) (tic_toc[tic_toc_size-1] - tic_toc[0])/1000000);
  cout << "fraction_" << fun_name << ",";  
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc - percent_tic_toc_last << ", ";
    percent_tic_toc_last = percent_tic_toc;
  }
  cout << "\ntime_" << fun_name << ",";
  double time_tic_toc_last = 0;
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc*dtime - time_tic_toc_last << ", ";
    time_tic_toc_last = percent_tic_toc*dtime;
  }
  cout << "\ntotal_time_" << fun_name << " " << dtime << "\n";  
}


/*

void App::publishStereo(const sensor_msgs::ImageConstPtr& l_image,
    const sensor_msgs::CameraInfoConstPtr& l_cam_info,
    const sensor_msgs::ImageConstPtr& r_image,
    const sensor_msgs::CameraInfoConstPtr& r_cam_info, std::string camera_out){
  
  #if DO_TIMING_PROFILE
    std::vector<int64_t> tic_toc;
    tic_toc.push_back(_timestamp_now());
  #endif
  
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  int isize = l_image->width*l_image->height;

  //cout << l_image->width << " w | h " << l_image->height << "\n";
  //cout << l_image->encoding << " q\n";

  if (l_image->encoding.compare("mono8") == 0){
    #if DO_TIMING_PROFILE
      tic_toc.push_back(_timestamp_now());
    #endif
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (l_image->width*l_image->height));
    #if DO_TIMING_PROFILE
      tic_toc.push_back(_timestamp_now());
    #endif
  }else{
    cout << l_image->encoding << " image encoded not supported, not publishing\n";
    cout << camera_out << "\n";
    return;
  }
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
    
  //int64_t tic = _timestamp_now();  
  if (1==1){
    int jpeg_quality_=90;
    imgutils_->jpegImageThenSend(stereo_data, current_utime, 
                l_image->width, 2*l_image->height, jpeg_quality_, camera_out);
  }else{
    stereo_msg_out_.utime =current_utime;
    stereo_msg_out_.width =l_image->width;
    stereo_msg_out_.height =2*l_image->height;
    stereo_msg_out_.nmetadata =0;
    stereo_msg_out_.row_stride=l_image->width;
    stereo_msg_out_.pixelformat =bot_core::image_t::PIXEL_FORMAT_GRAY;
    stereo_msg_out_.size =2*isize;
    stereo_msg_out_.data.assign(stereo_data, stereo_data + ( 2*isize));
    lcm_publish_.publish(camera_out, &stereo_msg_out_);
  }
  //std::cout << "pub: " <<  float((_timestamp_now() - tic)*1E-6) << "\n";;  
  
  // As a convenience also publish the left image:
  //send_image(l_image, "CAMERALEFT" );

  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"stereo");
  #endif    
}

*/

// typical contents: 640 x 480 images (out of date)
// points: xyz
// channels: rgb u v
int l_counter =0;
void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){

  
  int64_t current_utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  //if ( float(current_utime - head_left_last_utime_ ) > float(1E6/head_stereo_publish_fps_ ) ){
    //std::cout << "send left head\n";
    head_left_last_utime_ = current_utime;
    send_image(msg, "CAMERALEFT");
      
    if (l_counter%30 ==0){
      std::cout << "LIMG " << l_counter << "\n";
    }  
    l_counter++;    
    
    
  //}else{
    //std::cout << "dont send left head\n";
  //}  
  
  
  
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
}

void App::end_effector_sensors_cb(const atlas_msgs::ForceTorqueSensorsConstPtr& msg){
  end_effector_sensors_ = *msg;
}

int gt_counter =0;
void App::ground_truth_odom_cb(const nav_msgs::OdometryConstPtr& msg){
  if (gt_counter%200 ==0){
    std::cout << "GRTH " << gt_counter << "\n";
  }  
  gt_counter++;

  ground_truth_odom_ = *msg;
  init_recd_[0] =true;
}

/// Locally cache the joint states:
int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  if (js_counter%500 ==0){
    std::cout << "J ST " << js_counter << "\n";
  }  
  js_counter++;
  
  
  robot_joint_states_ = *msg; 
  init_recd_[1] =true; 
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  
  publishRobotState(joint_utime);
  
  drc::utime_t utime_msg;
  utime_msg.utime = joint_utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg);

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

int scan_counter=0;
void App::rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  if (scan_counter%80 ==0){
    std::cout << "SCAN " << scan_counter << "\n";
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
  string mode = "robot";
  bool control_output = true;// by default
  bool send_ground_truth = false;  
  bool send_head_cameras = false;
  bool send_hand_cameras = false;

  /*  ConciseArgs parser(argc, argv, "ros2lcm");
  parser.add(mode, "m", "mode", "Mode: robot, hands");
  parser.add(control_output, "c", "control_output", "Publish control message");
  parser.add(send_ground_truth, "g", "send_ground_truth", "Listen for and include GT odom");
  parser.add(send_head_cameras, "s", "send_head_cameras", "Send cameras in head/stereohead");
  parser.add(send_hand_cameras, "l", "send_hand_cameras", "Send cameras in hand/limbs");
  parser.parse();
  cout << "Publish Mode: " << mode << "\n";   
  cout << "Publish Control Messages: " << control_output << "\n";   
  cout << "Listen for and publish ground_truth_odom: " << send_ground_truth << " [control mode only]\n";   
  cout << "Publish Hand Camera Messages: " << send_hand_cameras << "\n";   
*/
  
  std::string mode_argument;
  if (argc >= 2){
     mode_argument = argv[1];
  }else {
    ROS_ERROR("Need to have another arguement in the launch file");
  }

  if (mode_argument.compare("vrc_cheats_enabled") == 0){
    send_ground_truth = true;
  }else if (mode_argument.compare("vrc_cheats_disabled") == 0){
    send_ground_truth = false;    
  }else {
    ROS_ERROR("mode_argument not understood");
    std::cout << mode_argument << " is not understood\n";
    exit(-1);
  }

  
  
  ros::init(argc, argv, "ros2lcm");
/*
  if(control_output){
  }else if(send_head_cameras){
    ros::init(argc, argv, "ros2lcm_head_cams");
  }else if(send_hand_cameras){
    ros::init(argc, argv, "ros2lcm_hand_cams");
  }else{
    cout << "Please choose a mode: control_output, send_head_cameras or send_hand_cameras\n";
    return -1;
  } */
  
  ros::CallbackQueue local_callback_queue;
  ros::NodeHandle nh;
//  nh.setCallbackQueue(&local_callback_queue);
  
  App *app = new App(nh, mode, control_output, send_ground_truth, send_head_cameras, send_hand_cameras);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("Control Translator: [%s]",  mode_argument.c_str());
  
  ros::spin();
//  while (ros::ok()){
//    local_callback_queue.callAvailable(ros::WallDuration(0.01));
  //}
  return 0;
}
