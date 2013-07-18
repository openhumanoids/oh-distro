#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"
#include <boost/shared_ptr.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <model-client/model-client.hpp>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ConciseArgs>

using namespace Eigen;
using namespace std;

class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &_lcm, bool _ground_truth_mode, bool _driving_mode);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  boost::shared_ptr<ModelClient> model_;
  
  void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * TRUE_state_msg);

  void handleImuMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::imu_t * msg);
  

private:
  void outputNoSensing(const drc::robot_state_t * TRUE_state_msg,
        KDL::Frame  T_body_head  );
  void outputDriving(const drc::robot_state_t * TRUE_state_msg,
        KDL::Frame  T_body_head  );
  void outputSensing(const drc::robot_state_t * TRUE_state_msg,
        KDL::Frame  T_body_head  );
  
  void sendPose(KDL::Frame pose, int64_t utime, std::string channel);
  void sendPose(drc::position_3d_t &origin, int64_t utime, std::string channel);

  boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
  KDL::Frame  _T_world_head;
  bool _ground_truth_mode;
  bool _driving_mode;
  
  drc::imu_t last_imu;
  /// used for Trigger based message demand:
  drc::robot_state_t _last_est_state;    
  //void sendTriggerOutput();
  
  BotParam* botparam_;
  BotFrames* botframes_;
  bot::frames* botframes_cpp_;
  
  bool got_imu_ ;
  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &_lcm, bool _ground_truth_mode, bool _driving_mode):
    _lcm(_lcm),got_imu_(false), _ground_truth_mode(_ground_truth_mode), _driving_mode(_driving_mode){
  botparam_ = bot_param_new_from_server(_lcm->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(_lcm->getUnderlyingLCM(), botparam_);
      
  _T_world_head = KDL::Frame::Identity();  
  // Get URDF string - this is instead of doing a URDF handler
  // These seems to need to be before the other subscribers otherwise
  // handleRobotStateMsg will be handled before we get the client    
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(_lcm->getUnderlyingLCM(), 0));

  _lcm->subscribe("TRUE_ROBOT_STATE", &StatePub::handleRobotStateMsg, this); //
  
  _lcm->subscribe("TORSO_IMU", &StatePub::handleImuMsg, this); //
  
  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(  model_->getURDFString() ,tree)){
    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
    return;
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  _last_est_state.utime = 0; // used to indicate no message recieved yet
  
}

void StatePub::handleImuMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::imu_t * msg){
  
  //double 
  //imu_orient_ msg->orientation;
  last_imu = *msg;
  got_imu_ =true;
}



void StatePub::sendPose(drc::position_3d_t &origin, int64_t utime, std::string channel){
  bot_core::pose_t pose_msg;
  pose_msg.utime = utime;
  pose_msg.pos[0] = origin.translation.x;
  pose_msg.pos[1] = origin.translation.y;
  pose_msg.pos[2] = origin.translation.z;
  pose_msg.orientation[0] = origin.rotation.w;
  pose_msg.orientation[1] = origin.rotation.x;
  pose_msg.orientation[2] = origin.rotation.y;
  pose_msg.orientation[3] = origin.rotation.z;
  _lcm->publish(channel, &pose_msg);
}

void StatePub::sendPose(KDL::Frame pose, int64_t utime, std::string channel){
  bot_core::pose_t pose_msg;
  pose_msg.utime = utime;
  pose_msg.pos[0] = pose.p[0];
  pose_msg.pos[1] = pose.p[1];
  pose_msg.pos[2] = pose.p[2];
  pose.M.GetQuaternion(pose_msg.orientation[1], pose_msg.orientation[2],
                              pose_msg.orientation[3], pose_msg.orientation[0]);  
  _lcm->publish(channel, &pose_msg);
}




void StatePub::outputDriving(const drc::robot_state_t * TRUE_state_msg,
  KDL::Frame  T_body_head){
  drc::robot_state_t msgout;
  msgout= *TRUE_state_msg;
  
  ///<!---->
  
  msgout.origin_position.translation.x=0;
  msgout.origin_position.translation.y=0;
  //z might need to change 
  msgout.origin_position.translation.z=1.059; 

  double quat[4] = {last_imu.orientation[0], last_imu.orientation[1], last_imu.orientation[2], last_imu.orientation[3]};
  
    /*{msgout.origin_position.rotation.w, msgout.origin_position.rotation.x, msgout.origin_position.rotation.y, 
  msgout.origin_position.rotation.z};*/
  double rpy[3] = {0};
  bot_quat_to_roll_pitch_yaw(quat, rpy);
  rpy[2] = 0;
  double quat_new[4];
  bot_roll_pitch_yaw_to_quat(rpy, quat_new);

  msgout.origin_position.rotation.w = quat_new[0];
  msgout.origin_position.rotation.x = quat_new[1];
  msgout.origin_position.rotation.y = quat_new[2];
  msgout.origin_position.rotation.z = quat_new[3];
  
  // this is the assumed height of the pelvis from the ground. 
  // this is paired with NOT running -g in drc-joint-frames to draw the height of the robot at z but with x and y =0
  // maintained by sachi
  _lcm->publish("EST_ROBOT_STATE", &msgout);
  sendPose(msgout.origin_position, msgout.utime, "POSE_BODY");
    
  // Infer the Robot's head position from the ground truth root world pose
  KDL::Frame T_world_body, T_world_head;
  T_world_body.p[0]= msgout.origin_position.translation.x;
  T_world_body.p[1]= msgout.origin_position.translation.y;
  T_world_body.p[2]= msgout.origin_position.translation.z;

  //take out the yaw in the message 
  
  

  /*fprintf(stderr, "Quat : %f, %f,%f,%f => %f,%f,%f,%f\n", 
          quat[0], quat[1], quat[2], quat[3], 
          quat_new[0], quat_new[1], quat_new[2], quat_new[3]);*/
  T_world_body.M =  KDL::Rotation::Quaternion(msgout.origin_position.rotation.x, msgout.origin_position.rotation.y, 
                                              msgout.origin_position.rotation.z, msgout.origin_position.rotation.w);

  //T_world_body.M =  KDL::Rotation::Quaternion(quat_new[1], quat_new[2], quat_new[3], quat_new[0]);

  T_world_head = T_world_body * T_body_head; 
  sendPose(T_world_head, msgout.utime, "POSE_HEAD");   
  sendPose(T_world_head, msgout.utime, "POSE_HEAD_TRUE"); // courtesy publish
  
  // Keep this for the trigger:
  _last_est_state = msgout;  
}




void StatePub::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
  const std::string& chan, const drc::robot_state_t * TRUE_state_msg){
  
  if (!got_imu_ ){
    return; 
  }
  
  // This line rate limits the entire system to 200Hz:
  if ( TRUE_state_msg->utime - _last_est_state.utime < 4500 ){
    return;
  }

  
  // call a routine that calculates the transforms the joint_state_t* TRUE_state_msg.
  map<string, double> jointpos_in;
  for (int i=0; i< TRUE_state_msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(TRUE_state_msg->joint_name[i], TRUE_state_msg->joint_position[i]));
  map<string, KDL::Frame > cartpos_out;

  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines the absolute transforms with respect to robot origin.
  //Otherwise returns relative transforms between joints.

  kinematics_status =  fksolver_->JntToCart(jointpos_in,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }

  // PRINTS THE VISUAL PROPERTIES OF ALL LINKS THAT HAVE A VISUAL ELEMENT DEFINED IN THE URDF FILE
  map<string, KDL::Frame>::const_iterator transform_it;

  KDL::Frame  T_body_head;
  transform_it=cartpos_out.find("head");
  T_body_head = KDL::Frame::Identity();
  if(transform_it!=cartpos_out.end()){// fk cart pos exists
    T_body_head= transform_it->second;
  }else{
    std::cout<< "fk position does not exist" <<std::endl;
  }
  
  KDL::Frame T_head_body;
  T_head_body = T_body_head.Inverse();
  bot_core::rigid_transform_t tf;
  tf.utime = TRUE_state_msg->utime;
  tf.trans[0] = T_head_body.p[0];
  tf.trans[1] = T_head_body.p[1];
  tf.trans[2] = T_head_body.p[2];
  T_head_body.M.GetQuaternion(tf.quat[1], tf.quat[2], tf.quat[3], tf.quat[0]);
  _lcm->publish("HEAD_TO_BODY", &tf);   
  
   outputDriving(TRUE_state_msg, T_body_head);
}


int main (int argc, char ** argv){
//  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool ground_truth_mode=false;
  bool driving_mode=true;
/*  parser.add(ground_truth_mode, "g", "ground_truth_mode", "Use Ground Truth");
  parser.add(driving_mode, "d", "driving_mode", "Use Driving Mode [xyz=0]");
  parser.parse();
  cout << "ground_truth_mode is: " << ground_truth_mode << "\n"; 
  cout << "driving_mode is: " << driving_mode << "\n"; 
  if (driving_mode && ground_truth_mode){
    std::cout << "please use either driving_mode or ground_truth_mode but not both\n";
   exit(-1); 
  }
*/

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm, ground_truth_mode,driving_mode);
  cout << "StatePub ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
