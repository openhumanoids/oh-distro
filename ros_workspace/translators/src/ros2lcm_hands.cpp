// Selective ros2lcm translator for hardware driver related ros messages.
// takes in arbitrary ros messages and publishes hand_state_t messages on 
// respective channels: SANDIA_LEFT_STATE/SANDIA_RIGHT_STATE/IROBOT_LEFT_STATE/IROBOT_RIGHT_STATE

// the sandia hand driver publishes ros messages at a finger level. They are appended here to form the hand sate message.

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>


#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sandia_hand_msgs/RawFingerState.h>
#include <sandia_hand_msgs/RawPalmState.h>
#include <handle_msgs/HandleSensors.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace std;

class App{
public:
  App(ros::NodeHandle node_,bool dumb_fingers);
  ~App();

private:
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  std::vector<std::string> sandiaJointNames;
  std::vector<std::string> irobotJointNames;

  // msg cache  
  sandia_hand_msgs::RawFingerState  sandia_l_hand_finger_0_state_,sandia_l_hand_finger_1_state_,sandia_l_hand_finger_2_state_,sandia_l_hand_finger_3_state_;
  sandia_hand_msgs::RawFingerState  sandia_r_hand_finger_0_state_,sandia_r_hand_finger_1_state_,sandia_r_hand_finger_2_state_,sandia_r_hand_finger_3_state_; 
  
  
  sandia_hand_msgs::RawPalmState sandia_l_hand_palm_state_,sandia_r_hand_palm_state_;
  sandia_hand_msgs::RawPalmState sandia_l_hand_palm_state_filtered_,sandia_r_hand_palm_state_filtered_;

  double sandia_l_hand_palm_diffstate_[32], sandia_l_hand_palm_origstate_[32];
  double sandia_r_hand_palm_diffstate_[32], sandia_r_hand_palm_origstate_[32];
  
  // sandia hand publishes raw finger state on separate messages. There is also cal_state, but it clear what that adds.
  ros::Subscriber  sandia_l_hand_finger_0_state_sub_, sandia_l_hand_finger_1_state_sub_, sandia_l_hand_finger_2_state_sub_, sandia_l_hand_finger_3_state_sub_,sandia_l_hand_palm_state_sub_;
  ros::Subscriber  sandia_r_hand_finger_0_state_sub_, sandia_r_hand_finger_1_state_sub_, sandia_r_hand_finger_2_state_sub_, sandia_r_hand_finger_3_state_sub_,sandia_r_hand_palm_state_sub_;  
 
  //Irobot hand subcribers and cache
  handle_msgs::HandleSensors irobot_l_hand_state_,irobot_r_hand_state_;
  ros::Subscriber  irobot_l_hand_joint_states_sub_, irobot_r_hand_joint_states_sub_; 
  
  void sandia_l_hand_finger_0_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  void sandia_l_hand_finger_1_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  void sandia_l_hand_finger_2_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  void sandia_l_hand_finger_3_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  
  void sandia_r_hand_finger_0_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  void sandia_r_hand_finger_1_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  void sandia_r_hand_finger_2_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  void sandia_r_hand_finger_3_state_cb(const sandia_hand_msgs::RawFingerStatePtr &msg);
  
  void sandia_l_hand_palm_state_cb(const sandia_hand_msgs::RawPalmStatePtr &msg);
  void sandia_r_hand_palm_state_cb(const sandia_hand_msgs::RawPalmStatePtr &msg);

  void appendSandiaFingerState(drc::hand_state_t& msg_out, sandia_hand_msgs::RawFingerState& msg_in,int finger_id);
  void publishSandiaHandState(int64_t utime_in,bool is_left);
  void publishHandStateOnSystemStatus(bool is_sandia, bool is_left);
  void publishSandiaRawTactile(int64_t utime_in,bool is_left);
  void publishSandiaCalibTactile(int64_t utime_in,bool is_left);
  
  void irobot_l_hand_state_cb(const handle_msgs::HandleSensorsPtr& msg);
  void irobot_r_hand_state_cb(const handle_msgs::HandleSensorsPtr& msg);
  
  // logic params
  bool init_recd_sandia_l_[5];
  bool init_recd_sandia_r_[5];
  bool init_recd_irobot_l_;
  bool init_recd_irobot_r_;
  
  bool dumb_fingers_;
  
  static const float l_hand_tactile_offset[32];
  static const float r_hand_tactile_offset[32];
  static const int L_SANDIA_TACTILE_THRESHOLD = 2000;
  static const int R_SANDIA_TACTILE_THRESHOLD = 2000;
  //////////////////////////////////////////////////////////////////////////
  
};

const float App::l_hand_tactile_offset[32] = {18129.210000,41816.800714,36852.560714,28905.737857,34026.741429,32183.515000,19285.279286,46726.437857,23855.493571,20807.182143,17698.305000,14489.970714,12628.599286,17087.452143,34570.179286,27990.855000,65535.000000,28409.595000,14924.827857,24572.253571,26622.789286,41039.437857,34172.065714,21753.195000,20706.341429,44552.820714,22502.300714,25812.208571,28436.020714,20312.915714,29820.766429,35515.578571};
const float App::r_hand_tactile_offset[32] = {27767.375572,38078.323971,25333.269758,46705.644677,46133.067929,23938.254082,30610.151535,30518.709340,33622.053560,35195.608099,34303.350098,39614.746571,34742.103854,33833.994121,15365.313521,33528.954931,44975.278250,27085.126061,26098.705421,33944.459830,50542.386675,24866.295885,65535.000000,32829.077074,29287.934683,35394.894840,36480.777270,39617.376878,31801.888961,45555.218158,34034.276290,41902.112345};


App::App(ros::NodeHandle node_, bool dumb_fingers) :
    node_(node_),dumb_fingers_(dumb_fingers){
  ROS_INFO("Initializing Sandia/Irobot Hands Translator (Not for simulation)");
  
  
 // Hand states
 sandia_l_hand_finger_0_state_sub_ = node_.subscribe(string("/sandia_hands/l_hand/finger_0/raw_state"), 100, &App::sandia_l_hand_finger_0_state_cb,this);
 sandia_l_hand_finger_1_state_sub_ = node_.subscribe(string("/sandia_hands/l_hand/finger_1/raw_state"), 100, &App::sandia_l_hand_finger_1_state_cb,this);
 sandia_l_hand_finger_2_state_sub_ = node_.subscribe(string("/sandia_hands/l_hand/finger_2/raw_state"), 100, &App::sandia_l_hand_finger_2_state_cb,this);
 sandia_l_hand_finger_3_state_sub_ = node_.subscribe(string("/sandia_hands/l_hand/finger_3/raw_state"), 100, &App::sandia_l_hand_finger_3_state_cb,this);
 
 sandia_r_hand_finger_0_state_sub_ = node_.subscribe(string("/sandia_hands/r_hand/finger_0/raw_state"), 100, &App::sandia_r_hand_finger_0_state_cb,this);
 sandia_r_hand_finger_1_state_sub_ = node_.subscribe(string("/sandia_hands/r_hand/finger_1/raw_state"), 100, &App::sandia_r_hand_finger_1_state_cb,this);
 sandia_r_hand_finger_2_state_sub_ = node_.subscribe(string("/sandia_hands/r_hand/finger_2/raw_state"), 100, &App::sandia_r_hand_finger_2_state_cb,this);
 sandia_r_hand_finger_3_state_sub_ = node_.subscribe(string("/sandia_hands/r_hand/finger_3/raw_state"), 100, &App::sandia_r_hand_finger_3_state_cb,this);
 
 sandia_l_hand_palm_state_sub_ = node_.subscribe(string("/sandia_hands/l_hand/palm/raw_state"), 100, &App::sandia_l_hand_palm_state_cb,this);
 sandia_r_hand_palm_state_sub_ = node_.subscribe(string("/sandia_hands/r_hand/palm/raw_state"), 100, &App::sandia_r_hand_palm_state_cb,this);

 //TODO: 
 // Irobot subcribers 
 irobot_l_hand_joint_states_sub_ = node_.subscribe(string("/irobot_hands/l_hand/sensors/raw"), 100, &App::irobot_l_hand_state_cb,this);
 irobot_r_hand_joint_states_sub_ = node_.subscribe(string("/irobot_hands/r_hand/sensors/raw"), 100, &App::irobot_r_hand_state_cb,this);
  
 
 
	sandiaJointNames.push_back("f0_j0");
	sandiaJointNames.push_back("f0_j1");
	sandiaJointNames.push_back("f0_j2");
	sandiaJointNames.push_back("f1_j0");	 
	sandiaJointNames.push_back("f1_j1");
	sandiaJointNames.push_back("f1_j2");
	sandiaJointNames.push_back("f2_j0");
	sandiaJointNames.push_back("f2_j1");
	sandiaJointNames.push_back("f2_j2");
	sandiaJointNames.push_back("f3_j0");	 
	sandiaJointNames.push_back("f3_j1");
	sandiaJointNames.push_back("f3_j2");
	
	
	irobotJointNames.push_back("finger[0]/joint_base_rotation");	 
	irobotJointNames.push_back("finger[0]/joint_base");
	irobotJointNames.push_back("finger[0]/joint_flex");
	irobotJointNames.push_back("finger[1]/joint_base_rotation");	 
	irobotJointNames.push_back("finger[1]/joint_base");
	irobotJointNames.push_back("finger[1]/joint_flex");
	irobotJointNames.push_back("finger[2]/joint_base");
	irobotJointNames.push_back("finger[2]/joint_flex");

	
	init_recd_sandia_l_[0]=false;
	init_recd_sandia_l_[1]=false;
	init_recd_sandia_l_[2]=false;
	init_recd_sandia_l_[3]=false;
	init_recd_sandia_l_[4]=false;
	init_recd_sandia_r_[0]=false;
	init_recd_sandia_r_[1]=false;
	init_recd_sandia_r_[2]=false;
	init_recd_sandia_r_[3]=false;	
	init_recd_sandia_r_[4]=false;
  init_recd_irobot_l_ = false;
  init_recd_irobot_r_ = false;
};

App::~App()  {
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

//----------------------------------------------------------------------------
// CALLBACKS FOR SANDIA HAND STATE
//----------------------------------------------------------------------------


void App::sandia_l_hand_palm_state_cb(const sandia_hand_msgs::RawPalmStatePtr& msg)
{
 if(!init_recd_sandia_l_[4]){
   init_recd_sandia_l_[4]=true;
   sandia_l_hand_palm_state_= *msg;
 }
 else
 {
  for(size_t i=0;i<msg->palm_tactile.size();i++)
  {
    // take and store the deriv of the signal against the running average
    double diff_val;
    diff_val=msg->palm_tactile[i]-sandia_l_hand_palm_state_.palm_tactile[i];
    sandia_l_hand_palm_diffstate_[i]=diff_val;
    sandia_l_hand_palm_origstate_[i]=msg->palm_tactile[i]-l_hand_tactile_offset[i];
// low pass filter the raw signal
    double old_val, alpha, new_val;
     old_val = sandia_l_hand_palm_state_.palm_tactile[i]; 
    alpha=0.1;
    new_val = alpha*msg->palm_tactile[i]+(1-alpha)*old_val;
    sandia_l_hand_palm_state_.palm_tactile[i] = new_val;
  }
 }

  int64_t utime = _timestamp_now();
  publishSandiaRawTactile(utime,true); 
  publishSandiaCalibTactile(utime,true);
}

//----------------------------------------------------------------------------
void App::sandia_r_hand_palm_state_cb(const sandia_hand_msgs::RawPalmStatePtr& msg)
{
 if(!init_recd_sandia_r_[4]){
   init_recd_sandia_r_[4]=true;
   sandia_r_hand_palm_state_= *msg;
 }
 else
 {
  for(size_t i=0;i<msg->palm_tactile.size();i++)
  {
      // take and store the deriv of the signal against the running average
    double diff_val;
    diff_val=msg->palm_tactile[i]-sandia_r_hand_palm_state_.palm_tactile[i];
    sandia_r_hand_palm_diffstate_[i]=diff_val;
    sandia_r_hand_palm_origstate_[i]=msg->palm_tactile[i]-r_hand_tactile_offset[i];
      // low pass filter the raw signal
    double old_val, alpha, new_val, new_val2;
    old_val = sandia_r_hand_palm_state_.palm_tactile[i];
    alpha=0.1;
    new_val = alpha*msg->palm_tactile[i]+(1-alpha)*old_val;
    sandia_r_hand_palm_state_.palm_tactile[i] = new_val;  // lhs type?
  }
 }

  int64_t utime = _timestamp_now();
  publishSandiaRawTactile(utime,false);
  publishSandiaCalibTactile(utime,false);
}

//----------------------------------------------------------------------------
void App::publishSandiaRawTactile(int64_t utime_in,bool is_left)
{

  drc:: raw_tactile_t msg_out;
  msg_out.utime = utime_in; // from nsec to usec
  
  msg_out.n_f0 = 0;
  msg_out.n_f1 = 0;
  msg_out.n_f2 = 0;
  msg_out.n_f3 = 0;

  if(is_left)
  {
     msg_out.n_palm=sandia_l_hand_palm_state_.palm_tactile.size();
     for (std::vector<int>::size_type i = 0; i < msg_out.n_palm; i++)  
     {  
        float val;
        val = (float) sandia_l_hand_palm_origstate_[i];   // use float, if using short it will overflow, source signal from ROS is u16int but LCM does not have unsigned short type
        msg_out.palm.push_back(val); // msg_out stores short
     }
     lcm_publish_.publish("SANDIA_LEFT_RAW_TACTILE_STATE", &msg_out); 
  } 
  else
  {
     msg_out.n_palm=sandia_r_hand_palm_state_.palm_tactile.size();
     for (std::vector<int>::size_type i = 0; i < msg_out.n_palm; i++)  
     {
        float val;
        val = (float)sandia_r_hand_palm_origstate_[i];
        msg_out.palm.push_back(val); // msg_out stores short
     }
     lcm_publish_.publish("SANDIA_RIGHT_RAW_TACTILE_STATE", &msg_out); 
  }
  
}
//----------------------------------------------------------------------------
void App::publishSandiaCalibTactile(int64_t utime_in,bool is_left)
{

  drc:: hand_tactile_state_t msg_out;
  msg_out.utime = utime_in; // from nsec to usec


  double vmax=0; double vmin=0;
  if(is_left)
  {
    
    for (std::vector<int>::size_type i = 0; i < 32; i++){
	     double v = (double) sandia_l_hand_palm_origstate_[i]; 
       vmax = std::max(vmax,v);
       vmin = std::min(vmin,v);
    }
    // take the max - min instead of sum to avoid value run off issues. (discuss with peter)
     msg_out.signal = (float)fabs(vmax-vmin);
     msg_out.touched =  msg_out.signal > L_SANDIA_TACTILE_THRESHOLD;
     lcm_publish_.publish("SANDIA_LEFT_TACTILE_STATE", &msg_out); 
  } 
  else
  {
    for (std::vector<int>::size_type i = 0; i < 32; i++){
	     double v = (double) sandia_r_hand_palm_origstate_[i];
        vmax = std::max(vmax,v);
        vmin = std::min(vmin,v);
    }
    // take the max-min instead of sum to avoid value run off issues. (discuss with peter)
    msg_out.signal = (float)fabs(vmax-vmin); 
    msg_out.touched =  msg_out.signal > R_SANDIA_TACTILE_THRESHOLD;
    lcm_publish_.publish("SANDIA_RIGHT_TACTILE_STATE", &msg_out); 
  }
  
}
//----------------------------------------------------------------------------
void App::sandia_l_hand_finger_0_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_l_[0])
   init_recd_sandia_l_[0]=true;
 sandia_l_hand_finger_0_state_= *msg;
}
void App::sandia_l_hand_finger_1_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_l_[1])
   init_recd_sandia_l_[1]=true;
 sandia_l_hand_finger_1_state_= *msg;
}
void App::sandia_l_hand_finger_2_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_l_[2])
   init_recd_sandia_l_[2]=true;
 sandia_l_hand_finger_2_state_= *msg;
}
void App::sandia_l_hand_finger_3_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_l_[3]){
   init_recd_sandia_l_[3]=true;   
   //publishHandStateOnSystemStatus(bool is_sandia, bool is_left)
   publishHandStateOnSystemStatus(true, true);
  }
 sandia_l_hand_finger_3_state_= *msg;
 int64_t utime = _timestamp_now();//has no header::(int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
 bool is_left = true;
 publishSandiaHandState(utime,is_left);
}
//----------------------------------------------------------------------------
void App::sandia_r_hand_finger_0_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_r_[0])
   init_recd_sandia_r_[0]=true;
 sandia_r_hand_finger_0_state_= *msg;
}
void App::sandia_r_hand_finger_1_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_r_[1])
   init_recd_sandia_r_[1]=true;
 sandia_r_hand_finger_1_state_= *msg;
}
void App::sandia_r_hand_finger_2_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_r_[2])
   init_recd_sandia_r_[2]=true;
 sandia_r_hand_finger_2_state_= *msg;
}
void App::sandia_r_hand_finger_3_state_cb(const sandia_hand_msgs::RawFingerStatePtr& msg)
{
 if(!init_recd_sandia_r_[3]){
   init_recd_sandia_r_[3]=true;
   //publishHandStateOnSystemStatus(bool is_sandia, bool is_left)
   publishHandStateOnSystemStatus(true, false);
  }

 sandia_r_hand_finger_3_state_= *msg;
 int64_t utime = _timestamp_now();// has no header::(int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
 bool is_left = false;
 publishSandiaHandState(utime,is_left);
}

//----------------------------------------------------------------------------
void App::publishSandiaHandState(int64_t utime_in,bool is_left){
  // If haven't got all four sandia finger states, exit:
  if((!init_recd_sandia_l_[0])&&(is_left))
    return;
  if((!init_recd_sandia_l_[1])&&(is_left))
    return;
  if((!init_recd_sandia_l_[2])&&(is_left))
    return;
  if((!init_recd_sandia_l_[3])&&(is_left))
    return;
    
  if((!init_recd_sandia_r_[0])&&(!is_left))
    return;
  if((!init_recd_sandia_r_[1])&&(!is_left))
    return;
  if((!init_recd_sandia_r_[2])&&(!is_left))
    return;
  if((!init_recd_sandia_r_[3])&&(!is_left))
    return;  

  drc::hand_state_t msg_out;
  msg_out.utime = utime_in; // from nsec to usec
  
  msg_out.num_joints = sandiaJointNames.size();
  
  for (std::vector<int>::size_type i = 0; i < sandiaJointNames.size(); i++)  {
    std::string name;
    if(is_left)
      name ="left_"+sandiaJointNames[i];
    else
      name ="right_"+sandiaJointNames[i];  
    msg_out.joint_name.push_back(name);      
    msg_out.joint_position.push_back(0);      
    msg_out.joint_velocity.push_back(0);
    msg_out.joint_effort.push_back(0);
  }  
   
  if(is_left){
   appendSandiaFingerState(msg_out, sandia_l_hand_finger_0_state_,0);
   appendSandiaFingerState(msg_out, sandia_l_hand_finger_1_state_,1);
   appendSandiaFingerState(msg_out, sandia_l_hand_finger_2_state_,2);
   appendSandiaFingerState(msg_out, sandia_l_hand_finger_3_state_,3);
   lcm_publish_.publish("SANDIA_LEFT_STATE", &msg_out); 
  } 
  else{
   appendSandiaFingerState(msg_out, sandia_r_hand_finger_0_state_,0);
   appendSandiaFingerState(msg_out, sandia_r_hand_finger_1_state_,1);
   appendSandiaFingerState(msg_out, sandia_r_hand_finger_2_state_,2);
   appendSandiaFingerState(msg_out, sandia_r_hand_finger_3_state_,3);
   lcm_publish_.publish("SANDIA_RIGHT_STATE", &msg_out); 
  }
   
}
//----------------------------------------------------------------------------
void App::appendSandiaFingerState(drc::hand_state_t& msg_out, sandia_hand_msgs::RawFingerState& msg_in,int finger_id){
  
    // calculate joint angles based on hall sensor offsets
   double H2R, R0_INV,R1_INV,R2_INV,CAPSTAN_RATIO;
    H2R = 3.14159 * 2.0 / 36.0;// hall state to radians: 18 pole pairs
    R0_INV = 1.0 / 231.0;
    R1_INV = 1.0 / 196.7;
    R2_INV = 1.0 / 170.0;
    CAPSTAN_RATIO = 0.89;
    msg_out.joint_position[0+finger_id*3] = -H2R * R0_INV * ( msg_in.hall_pos[0] );
    msg_out.joint_position[1+finger_id*3] =  H2R * R1_INV * ( msg_in.hall_pos[1] + CAPSTAN_RATIO*msg_in.hall_pos[0] );
    msg_out.joint_position[2+finger_id*3] =  H2R * R2_INV * ( msg_in.hall_pos[2] - msg_in.hall_pos[1] - CAPSTAN_RATIO*2*msg_in.hall_pos[0] );
    msg_out.joint_velocity[0+finger_id*3] = 0;
    msg_out.joint_velocity[1+finger_id*3] = 0;
    msg_out.joint_velocity[2+finger_id*3] = 0;
    msg_out.joint_effort[0+finger_id*3] = 0;//msg_in.fmcb_effort[0];
    msg_out.joint_effort[1+finger_id*3] = 0;//msg_in.fmcb_effort[1];
    msg_out.joint_effort[2+finger_id*3] = 0;//msg_in.fmcb_effort[2];
}

//----------------------------------------------------------------------------
// CALLBACKS FOR IROBOT HAND STATE
//----------------------------------------------------------------------------
void App::irobot_l_hand_state_cb(const handle_msgs::HandleSensorsPtr& msg)
{
 if(!init_recd_irobot_l_){
   init_recd_irobot_l_=true;
   //publishHandStateOnSystemStatus(bool is_sandia, bool is_left)
   publishHandStateOnSystemStatus(false, true);
  }
   
  irobot_l_hand_state_ = *msg;
  drc::hand_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  msg_out.num_joints = irobotJointNames.size();
  for (std::vector<int>::size_type i = 0; i < irobotJointNames.size(); i++)  {
    std::string name;
    name ="left_"+irobotJointNames[i];
    msg_out.joint_name.push_back(name);      
    msg_out.joint_position.push_back(0);      
    msg_out.joint_velocity.push_back(0);
    msg_out.joint_effort.push_back(0);
  }  
  /*
  finger[0]/joint_base_rotation and finger[1]/joint_base_rotation
  ==============================================================
  # The encoder on the F1 / F2 base rotation. 8.533 ticks per degree.
  # 768 ticks to rotate the fingers 90 degrees for a "T" grasp.
  # 512 ticks to rotate the fingers 60 degrees for a spherical grasp.
  */
  {
    double tick_to_radians = ((0.5*M_PI)/768);
    msg_out.joint_position[0]=tick_to_radians*msg->fingerSpread;
    msg_out.joint_position[3]=tick_to_radians*msg->fingerSpread;
  }
  
 // very inaccurate estimate of flexion via hall effect sensor for dumb fingers
  
  /*finger[0]/joint_base,finger[1]/joint_base and finger[2]/joint_base
  ==============================================================
  # The hall effect sensor on the finger motors.  
  # 24 counts per motor revolution x 300 motor revolutions for one full spindle
  # rotation.  
  # 3500 to put finger at approx. 90 degrees
  # 6000 to close finger gently
  # 7000 to close finger tightly
  #
  # [F1, F2, F3, F3 Ant.]
  int32[4] motorHallEncoder
  */
 if(dumb_fingers_)
 {
    double tick_to_radians = ((0.5*M_PI)/3500); 
    msg_out.joint_position[1]=tick_to_radians*msg->motorHallEncoder[0];
    msg_out.joint_position[4]=tick_to_radians*msg->motorHallEncoder[1]; 
    msg_out.joint_position[6]=tick_to_radians*msg->motorHallEncoder[2];
 }
 else
 {
 //TODO: USE PROXIMAL JOINT ENCODERS
 }
  lcm_publish_.publish("IROBOT_LEFT_STATE", &msg_out);  
}

//----------------------------------------------------------------------------
void App::irobot_r_hand_state_cb(const handle_msgs::HandleSensorsPtr& msg)
{
 if(!init_recd_irobot_r_){
   init_recd_irobot_r_=true;
   //publishHandStateOnSystemStatus(bool is_sandia, bool is_left)
   publishHandStateOnSystemStatus(false, false);
  }
  
  irobot_r_hand_state_ = *msg;
  drc::hand_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  msg_out.num_joints = irobotJointNames.size();
  for (std::vector<int>::size_type i = 0; i < irobotJointNames.size(); i++)  {
    std::string name;
    name ="right_"+irobotJointNames[i];
    msg_out.joint_name.push_back(name);      
    msg_out.joint_position.push_back(0);      
    msg_out.joint_velocity.push_back(0);
    msg_out.joint_effort.push_back(0);
  }  
  /*
  finger[0]/joint_base_rotation and finger[1]/joint_base_rotation
  ==============================================================
  # The encoder on the F1 / F2 base rotation. 8.533 ticks per degree.
  # 768 ticks to rotate the fingers 90 degrees for a "T" grasp.
  # 512 ticks to rotate the fingers 60 degrees for a spherical grasp.
  */
  {
    double tick_to_radians = ((0.5*M_PI)/768);
    msg_out.joint_position[0]=tick_to_radians*msg->fingerSpread;
    msg_out.joint_position[3]=tick_to_radians*msg->fingerSpread;
  }
  
 // very inaccurate estimate of flexion via hall effect sensor for dumb fingers
  
  /*finger[0]/joint_base,finger[1]/joint_base and finger[2]/joint_base
  ==============================================================
  # The hall effect sensor on the finger motors.  
  # 24 counts per motor revolution x 300 motor revolutions for one full spindle
  # rotation.  
  # 3500 to put finger at approx. 90 degrees
  # 6000 to close finger gently
  # 7000 to close finger tightly
  #
  # [F1, F2, F3, F3 Ant.]
  int32[4] motorHallEncoder
  */
 if(dumb_fingers_)
 {
    double tick_to_radians = ((0.5*M_PI)/3500); 
    msg_out.joint_position[1]=tick_to_radians*msg->motorHallEncoder[0];
    msg_out.joint_position[4]=tick_to_radians*msg->motorHallEncoder[1]; 
    msg_out.joint_position[6]=tick_to_radians*msg->motorHallEncoder[2];
 }
 else
 {
 //TODO: USE PROXIMAL JOINT ENCODERS
 }
  lcm_publish_.publish("IROBOT_RIGHT_STATE", &msg_out);  
}
//----------------------------------------------------------------------------
void App::publishHandStateOnSystemStatus(bool is_sandia, bool is_left)
{
  drc::system_status_t msg;
  msg.utime = _timestamp_now();
  msg.system = msg.GRASPING;
  msg.importance = 0x00;
  msg.frequency = 0x03;
  
  if((is_sandia)&&(is_left))
  msg.value = "LEFT SANDIA HAND ACTIVE: Receiving state messages";
  else if((is_sandia)&&(!is_left))
    msg.value = "RIGHT SANDIA HAND ACTIVE: Receiving state messages";
  else if((!is_sandia)&&(is_left))
    msg.value = "LEFT IROBOT HAND ACTIVE: Receiving state messages";
  else if((!is_sandia)&&(!is_left))
    msg.value = "RIGHT IROBOT HAND ACTIVE: Receiving state messages";
    
  lcm_publish_.publish("SYSTEM_STATUS", &msg); 

}
//----------------------------------------------------------------------------
int main(int argc, char **argv){

  ros::init(argc, argv, "ros2lcm_hands");
  ros::NodeHandle nh;
  bool dumb_fingers= true; //irobot hands are configured with dumb fingers
  App *app = new App(nh,dumb_fingers);
  std::cout << "ros2lcm_hands translator ready\n";
  ros::spin();
  return 0;
}
