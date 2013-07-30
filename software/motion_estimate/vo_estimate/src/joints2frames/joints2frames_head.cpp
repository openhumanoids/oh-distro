#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>       // std::numeric_limits

#include "joints2frames.hpp"
#include <ConciseArgs>

using namespace std;
using namespace boost;
using namespace boost::assign;


/////////////////////////////////////

joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &lcm_, bool show_labels_, bool show_triads_,
  bool standalone_head_, bool ground_height_):
          lcm_(lcm_), show_labels_(show_labels_), show_triads_(show_triads_),
          standalone_head_(standalone_head_), ground_height_(ground_height_){
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(6001,"Frames",5,1) );
  lcm_->subscribe("EST_ROBOT_STATE",&joints2frames::robot_state_handler,this);  
  
  last_ground_publish_utime_ =0;
  last_joint_publish_utime_ =0;
}

Eigen::Isometry3d DRCTransformToEigen(drc::transform_t tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.translation.x, tf.translation.y, tf.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(tf.rotation.w, tf.rotation.x, 
                                               tf.rotation.y, tf.rotation.z);
  tf_out.rotate(quat);    
  return tf_out;
}



void quat_to_euler_XXX(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

std::string print(Eigen::Isometry3d pose){
  std::stringstream ss;
  
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  double ypr[3];
  quat_to_euler_XXX(r, ypr[0], ypr[1], ypr[2]);
  
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<" | " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() << " | RPY "
       << ypr[2] <<", "<< ypr[1] <<", "<< ypr[0];
       
  return ss.str();
}


void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  
  // 0. Extract World Pose of body:
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  world_to_body.rotate(quat);    
    
  // 1. Solve for Forward Kinematics:
  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  map<string, drc::transform_t > cartpos_out;
  for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));

  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  
    
   
  // 2a. Determine the required BOT_FRAMES transforms:
  Eigen::Isometry3d head_to_hokuyo, head_to_left_optical;
  bool head_to_hokuyo_found =false;
  bool head_to_left_optical_found = false;

  //
  Eigen::Isometry3d head_to_pre_spindle, head_to_pre_spindle_cal_yaw, head_to_post_spindle;
  
  for( map<string, drc::transform_t>::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string joint2 = (*ii).first;
    if (   (*ii).first.compare( "head_hokuyo_frame" ) == 0 ){
      head_to_hokuyo = DRCTransformToEigen( (*ii).second );
      head_to_hokuyo_found=true;
    }else if(  (*ii).first.compare( "left_camera_optical_frame" ) == 0 ){
      head_to_left_optical = DRCTransformToEigen( (*ii).second );
      head_to_left_optical_found=true;
    }else if(  (*ii).first.compare( "pre_spindle" ) == 0 ){
      head_to_pre_spindle = DRCTransformToEigen( (*ii).second );
    }else if(  (*ii).first.compare( "pre_spindle_cal_yaw" ) == 0 ){
      head_to_pre_spindle_cal_yaw = DRCTransformToEigen( (*ii).second );
    }else if(  (*ii).first.compare( "post_spindle" ) == 0 ){ 
      head_to_post_spindle = DRCTransformToEigen( (*ii).second );
    }
  }
  
  
  
  {
//    std::cout << print(head_to_pre_spindle) << " head_to_pre_spindle\n";
  }
  
  
  
  
  
  
  {
  Eigen::Isometry3d pre_spindle_to_pre_spindle_cal_yaw = head_to_pre_spindle.inverse() * head_to_pre_spindle_cal_yaw ;
  std::stringstream ss2;
  print_Isometry3d(pre_spindle_to_pre_spindle_cal_yaw,ss2);
  std::cout << "pre_spindle_to_pre_spindle_cal_yaw: " << ss2.str() << "\n";
  std::cout << "pre_spindle_to_pre_spindle_cal_yaw: " << pre_spindle_to_pre_spindle_cal_yaw.rotation() << "\n";
  }
  
  {
//  Eigen::Isometry3d pre_spindle_cal_yaw_to_post_spindle = head_to_pre_spindle_cal_yaw.inverse() * head_to_post_spindle ;
//  std::stringstream ss2;
//  print_Isometry3d(pre_spindle_cal_yaw_to_post_spindle,ss2);
//  std::cout << "pre_spindle_cal_yaw_to_post_spindle: " << ss2.str() << "\n";
  
//  std::stringstream ss3;
  //print_Isometry3d(pre_spindle_cal_yaw_to_post_spindle.inverse(),ss3);
  //std::cout << "pre_spindle_cal_yaw_to_post_spindle inv: " << ss3.str() << "\n";
  }
  
  
  std::cout << "\n\n";
  
  
  
  
  
  ////////////////////////////////////////////  
  
  if (head_to_hokuyo_found && head_to_left_optical_found){
    Eigen::Isometry3d cam_to_hokuyo_frame = head_to_left_optical.inverse() * head_to_hokuyo ;
    
    bot_core::rigid_transform_t tf;
    tf.utime = msg->utime;
    tf.trans[0] = cam_to_hokuyo_frame.translation().x();
    tf.trans[1] = cam_to_hokuyo_frame.translation().y();
    tf.trans[2] = cam_to_hokuyo_frame.translation().z();
    Eigen::Quaterniond quat = Eigen::Quaterniond( cam_to_hokuyo_frame.rotation() );
    tf.quat[0] = quat.w();
    tf.quat[1] = quat.x();
    tf.quat[2] = quat.y();
    tf.quat[3] = quat.z();
    lcm_->publish("CAMERA_TO_SCAN_URDF", &tf);     
  }
  
  
  // 4. Loop through joints and extract world positions:
  if (!show_triads_){     return;    }
  int counter =msg->utime;  
  std::vector<Isometry3dTime> body_to_jointTs, world_to_jointsT;
  std::vector< int64_t > body_to_joint_utimes;
  std::vector< std::string > joint_names;
  for( map<string, drc::transform_t>::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string joint = (*ii).first;
    //cout << joint  << ": \n";
    joint_names.push_back( joint  );
    body_to_joint_utimes.push_back( counter);
    
    Eigen::Isometry3d body_to_joint;
    body_to_joint.setIdentity();
    body_to_joint.translation()  << (*ii).second.translation.x, (*ii).second.translation.y, (*ii).second.translation.z;
    Eigen::Quaterniond quat = Eigen::Quaterniond((*ii).second.rotation.w, (*ii).second.rotation.x, (*ii).second.rotation.y, (*ii).second.rotation.z);
    body_to_joint.rotate(quat);    
    Isometry3dTime body_to_jointT(counter, body_to_joint);
    body_to_jointTs.push_back(body_to_jointT);
    // convert to world positions
    Isometry3dTime world_to_jointT(counter, world_to_body*body_to_joint);
    world_to_jointsT.push_back(world_to_jointT);
    counter++;
  }
  
  pc_vis_->pose_collection_to_lcm_from_list(6001, world_to_jointsT); // all joints in world frame
  if (show_labels_)
    pc_vis_->text_collection_to_lcm(6002, 6001, "Frames [Labels]", joint_names, body_to_joint_utimes );    
}


int
main(int argc, char ** argv){
  string role = "robot";
  bool labels = false;
  bool triads = false;
  bool standalone_head = false;
  bool ground_height = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(role, "r", "role","Role - robot or base");
  opt.add(triads, "t", "triads","Frame Triads - show no not");
  opt.add(labels, "l", "labels","Frame Labels - show no not");
  opt.add(ground_height, "g", "ground", "Publish the grounded foot pose");
  opt.add(standalone_head, "s", "standalone_head","Standalone Sensor Head");
  opt.parse();
  if (labels){ // require triads if labels is to be published
    triads=true;
  }
  
  std::cout << "triads: " << triads << "\n";
  std::cout << "labels: " << labels << "\n";
  std::cout << "role: " << role << "\n";

  string lcm_url="";
  std::string role_upper;
  for(short i = 0; i < role.size(); ++i)
     role_upper+= (std::toupper(role[i]));
  if((role.compare("robot") == 0) || (role.compare("base") == 0) ){
    for(short i = 0; i < role_upper.size(); ++i)
       role_upper[i] = (std::toupper(role_upper[i]));
    string env_variable_name = string("LCM_URL_DRC_" + role_upper); 
    char* env_variable;
    env_variable = getenv (env_variable_name.c_str());
    if (env_variable!=NULL){
      //printf ("The env_variable is: %s\n",env_variable);      
      lcm_url = string(env_variable);
    }else{
      std::cout << env_variable_name << " environment variable has not been set ["<< lcm_url <<"]\n";     
      exit(-1);
    }
  }else{
    std::cout << "Role not understood, choose: robot or base\n";
    return 1;
  }
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM(lcm_url) );
  if(!lcm->good())
    return 1;  
  
  joints2frames app(lcm,labels,triads, standalone_head, ground_height);
  while(0 == lcm->handle());
  return 0;
}

