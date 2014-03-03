#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/robot_state_t.hpp"
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
  void handlePoseHeadMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
  //void triggerHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::data_request_t* msg);

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
  bool _initPoseHead;  
  bool _ground_truth_mode;
  bool _driving_mode;
  
  /// used for Trigger based message demand:
  drc::robot_state_t _last_est_state;    
  //void sendTriggerOutput();
  
  BotParam* botparam_;
  BotFrames* botframes_;
  bot::frames* botframes_cpp_;
  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &_lcm, bool _ground_truth_mode, bool _driving_mode):
    _lcm(_lcm),_initPoseHead(false), _ground_truth_mode(_ground_truth_mode), _driving_mode(_driving_mode){
  botparam_ = bot_param_new_from_server(_lcm->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(_lcm->getUnderlyingLCM(), botparam_);
      
  _T_world_head = KDL::Frame::Identity();  
  // Get URDF string - this is instead of doing a URDF handler
  // These seems to need to be before the other subscribers otherwise
  // handleRobotStateMsg will be handled before we get the client    
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(_lcm->getUnderlyingLCM(), 0));

  _lcm->subscribe("TRUE_ROBOT_STATE", &StatePub::handleRobotStateMsg, this); //
  _lcm->subscribe("POSE_HEAD",&StatePub::handlePoseHeadMsg,this);
  //_lcm->subscribe("TRIGGER_STATE",&StatePub::triggerHandler,this);
  
  // EST_ROBOT_STATE as received on the base from the shaper:
  _lcm->subscribe("EST_ROBOT_STATE_TX", &StatePub::handleRobotStateMsg, this); //
  
  
  
  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(  model_->getURDFString() ,tree)){
    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
    return;
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  _last_est_state.utime = 0; // used to indicate no message recieved yet
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

void StatePub::outputNoSensing(const drc::robot_state_t * TRUE_state_msg,
  KDL::Frame  T_body_head){
  
  
  drc::robot_state_t msgout;
  msgout= *TRUE_state_msg;
  _lcm->publish("EST_ROBOT_STATE", &msgout);
  sendPose(msgout.pose, msgout.utime, "POSE_BODY");
    
  // Infer the Robot's head position from the ground truth root world pose
  KDL::Frame T_world_body, T_world_head;
  T_world_body.p[0]= msgout.pose.translation.x;
  T_world_body.p[1]= msgout.pose.translation.y;
  T_world_body.p[2]= msgout.pose.translation.z;
  T_world_body.M =  KDL::Rotation::Quaternion(msgout.pose.rotation.x, msgout.pose.rotation.y, 
                                                msgout.pose.rotation.z, msgout.pose.rotation.w);
  T_world_head = T_world_body * T_body_head; 
  sendPose(T_world_head, msgout.utime, "POSE_HEAD");   
  sendPose(T_world_head, msgout.utime, "POSE_HEAD_TRUE"); // courtesy publish
  
  // Keep this for the trigger:
  _last_est_state = msgout;  
}



void StatePub::outputDriving(const drc::robot_state_t * TRUE_state_msg,
  KDL::Frame  T_body_head){
  drc::robot_state_t msgout;
  msgout= *TRUE_state_msg;
  msgout.pose.translation.x=0;
  msgout.pose.translation.y=0;
  //z might need to change 
  msgout.pose.translation.z=1.059; 

  double quat[4] = {msgout.pose.rotation.w, msgout.pose.rotation.x, msgout.pose.rotation.y, 
                    msgout.pose.rotation.z};
  double rpy[3] = {0};
  bot_quat_to_roll_pitch_yaw(quat, rpy);
  rpy[2] = 0;
  double quat_new[4];
  bot_roll_pitch_yaw_to_quat(rpy, quat_new);

  msgout.pose.rotation.w = quat_new[0];
  msgout.pose.rotation.x = quat_new[1];
  msgout.pose.rotation.y = quat_new[2];
  msgout.pose.rotation.z = quat_new[3];
  
  // this is the assumed height of the pelvis from the ground. 
  // this is paired with NOT running -g in drc-joint-frames to draw the height of the robot at z but with x and y =0
  // maintained by sachi
  _lcm->publish("EST_ROBOT_STATE", &msgout);
  sendPose(msgout.pose, msgout.utime, "POSE_BODY");
    
  // Infer the Robot's head position from the ground truth root world pose
  KDL::Frame T_world_body, T_world_head;
  T_world_body.p[0]= msgout.pose.translation.x;
  T_world_body.p[1]= msgout.pose.translation.y;
  T_world_body.p[2]= msgout.pose.translation.z;

  //take out the yaw in the message 
  
  

  /*fprintf(stderr, "Quat : %f, %f,%f,%f => %f,%f,%f,%f\n", 
          quat[0], quat[1], quat[2], quat[3], 
          quat_new[0], quat_new[1], quat_new[2], quat_new[3]);*/
  T_world_body.M =  KDL::Rotation::Quaternion(msgout.pose.rotation.x, msgout.pose.rotation.y, 
                                              msgout.pose.rotation.z, msgout.pose.rotation.w);

  //T_world_body.M =  KDL::Rotation::Quaternion(quat_new[1], quat_new[2], quat_new[3], quat_new[0]);

  T_world_head = T_world_body * T_body_head; 
  sendPose(T_world_head, msgout.utime, "POSE_HEAD");   
  sendPose(T_world_head, msgout.utime, "POSE_HEAD_TRUE"); // courtesy publish
  
  // Keep this for the trigger:
  _last_est_state = msgout;  
}



void StatePub::outputSensing(const drc::robot_state_t * TRUE_state_msg,
  KDL::Frame  T_body_head){
  
  // Infer the Robot's head position from the ground truth root world pose
  KDL::Frame T_world_body_GT, T_world_head_GT;
  T_world_body_GT.p[0]= TRUE_state_msg->pose.translation.x;
  T_world_body_GT.p[1]= TRUE_state_msg->pose.translation.y;
  T_world_body_GT.p[2]= TRUE_state_msg->pose.translation.z;
  T_world_body_GT.M =  KDL::Rotation::Quaternion(TRUE_state_msg->pose.rotation.x, TRUE_state_msg->pose.rotation.y, 
                                                TRUE_state_msg->pose.rotation.z, TRUE_state_msg->pose.rotation.w);
  T_world_head_GT = T_world_body_GT * T_body_head; 
  sendPose(T_world_head_GT, TRUE_state_msg->utime, "POSE_HEAD_TRUE");  
  
  
  if (!_initPoseHead){
    std::cout <<"Haven't received the first POSE_HEAD yet.\nRefusing to output EST_ROBOT_STATE ["<< TRUE_state_msg->utime <<"]\n";
    return; 
  }
  
  // Determine world position of root link using head position and relative transform
  KDL::Frame  T_world_body;
  T_world_body  = _T_world_head*T_body_head.Inverse();
  drc::position_3d_t body_origin;
  body_origin.translation.x = T_world_body.p[0];
  body_origin.translation.y = T_world_body.p[1];
  body_origin.translation.z = T_world_body.p[2];
  T_world_body.M.GetQuaternion(body_origin.rotation.x,body_origin.rotation.y,body_origin.rotation.z,body_origin.rotation.w);

  // EST is TRUE with sensor estimated position
  drc::robot_state_t msgout;
  msgout = *TRUE_state_msg;
  msgout.pose = body_origin;
  _lcm->publish("EST_ROBOT_STATE", &msgout);

  // Publish robot's root link position as a curtesy - prob no necessary:
  sendPose(body_origin, msgout.utime, "POSE_BODY");   
  // Keep this for the trigger:
  _last_est_state = msgout;
}

void StatePub::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
  const std::string& chan, const drc::robot_state_t * TRUE_state_msg){
  
  // This line rate limits the entire system to 200Hz:
  if ( TRUE_state_msg->utime - _last_est_state.utime < 4500 ){
    //std::cout << "skip TRS\n";
    return;
  }
  //std::cout << "send TRS\n";
    
  
  /* This prints out the joint ordering - so they can be copied to drc_robot.cfg and kept in sync:
  cout << "[";
  for (size_t i=0; i < TRUE_state_msg->num_joints; i++){
    cout << "\"" << TRUE_state_msg->joint_name[i]  << "\"" ;
    if (i < TRUE_state_msg->num_joints - 1)
      cout << ", ";
      
    if ( (i+1)%7==0)
      cout << "\n";
  }
  cout << "];\n";
  */
  
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
  
  // If not outputing the EST_ROBOT_STATE from here, 
  // Sensing mode is now provided by Motion Estimation Modules Directly
  if(! (_ground_truth_mode|| _driving_mode)  ){ // formerly the sensing mode
    return;
  }
  
  
    
  if(_ground_truth_mode  ){ // formerly est_robot_state_no_sensing
    outputNoSensing(TRUE_state_msg, T_body_head);
  }else if(_driving_mode){
    outputDriving(TRUE_state_msg, T_body_head);
  }
  //}else{
  //  outputSensing(TRUE_state_msg, T_body_head;)
  //}
}

void StatePub::handlePoseHeadMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg)  {
  _initPoseHead=true;
  _T_world_head.p[0]= msg->pos[0];
  _T_world_head.p[1]= msg->pos[1];
  _T_world_head.p[2]= msg->pos[2];
  _T_world_head.M =  KDL::Rotation::Quaternion(msg->orientation[1], msg->orientation[2], msg->orientation[3], msg->orientation[0]);
  // std::cout<< "head x,y,z in world frame:" <<_T_world_head.p[0] <<" , " <<_T_world_head.p[1] <<" , "<< _T_world_head.p[2] <<std::endl;
};


// This is now done in a different program "est_robot_state_to_minimal"
/*
// This function can also be used to publish on 
// a regular basis e.g. if set to publish at 1Hz
void StatePub::sendTriggerOutput(){
  
  if (_last_est_state.utime==0){     return;   } // if no msg recieved yet then ignore output command
  drc::minimal_robot_state_t msgout;
  msgout.utime = _last_est_state.utime;
  msgout.pose = _last_est_state.pose;
  msgout.num_joints = _last_est_state.num_joints;
  msgout.joint_position = _last_est_state.joint_position;
  _lcm->publish("EST_ROBOT_STATE_MINIMAL", &msgout);        
  cout << "Sending Triggered Message\n";
}

void StatePub::triggerHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::data_request_t* msg){
  sendTriggerOutput();
}
*/

int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool ground_truth_mode=false;
  bool driving_mode=false;
  parser.add(ground_truth_mode, "g", "ground_truth_mode", "Use Ground Truth");
  parser.add(driving_mode, "d", "driving_mode", "Use Driving Mode [xyz=0]");
  parser.parse();
  cout << "ground_truth_mode is: " << ground_truth_mode << "\n"; 
  cout << "driving_mode is: " << driving_mode << "\n"; 
  if (driving_mode && ground_truth_mode){
    std::cout << "please use either driving_mode or ground_truth_mode but not both\n";
   exit(-1); 
  }
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm, ground_truth_mode,driving_mode);
  cout << "StatePub ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
