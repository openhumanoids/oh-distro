#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>
#include <fstream>

#include "leg_odometry.hpp"
#include <path_util/path_util.h>


#include "common_conversions.hpp"

using namespace std;
using namespace boost;
using namespace boost::assign;

leg_odometry::leg_odometry(boost::shared_ptr<lcm::LCM> &lcm_subscribe_,  boost::shared_ptr<lcm::LCM> &lcm_publish_, const CommandLineConfig& cl_cfg_):
          lcm_subscribe_(lcm_subscribe_), lcm_publish_(lcm_publish_), cl_cfg_(cl_cfg_){
  
  if (cl_cfg_.config_filename == ""){
    botparam_ = bot_param_new_from_server(lcm_subscribe_->getUnderlyingLCM(), 0);
  }else{
    //std::string config_filename = "drc_robot_02.cfg";            
    std::string config_filename_full = std::string(getConfigPath()) +'/' + std::string(cl_cfg_.config_filename);
    botparam_ = bot_param_new_from_file(config_filename_full.c_str());
  }
            
  if (cl_cfg_.urdf_filename == ""){           
    model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_subscribe_->getUnderlyingLCM(), 0));
  }else{
    //std::string urdf_filename = "model_LH_RH.urdf";            
    std::string urdf_filename_full = std::string(getModelsPath()) +"/mit_gazebo_models/mit_robot/" + std::string(cl_cfg_.urdf_filename);
    model_ = boost::shared_ptr<ModelClient>(new ModelClient( urdf_filename_full  ));
  }

  leg_odometry_mode_ = bot_param_get_str_or_fail(botparam_, "state_estimator.leg_odometry_mode");            
  
  
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_publish_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1001,"Body Pose",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1002,"Primary Foot",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1003,"Secondary Foot",5,1) );
  
  
  bool log_data_files = false;
  float atlas_weight = 1400.0;
  
  foot_contact_logic_ = new TwoLegs::FootContact(log_data_files, atlas_weight);
  foot_contact_logic_->setStandingFoot( LEFTFOOT ); // Not sure that double states should be used, this should probably change TODO
  primary_foot_ = 0; // ie left
  leg_odo_init_ = false;
   
  verbose_ = 1;
  openLogFile();
}


void leg_odometry::openLogFile(){
  time_t rawtime;
  struct tm * timeinfo;  
  char buffer [80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);

  strftime (buffer,80,"/tmp/state-estimate-result-%Y-%m-%d-%H-%M.txt",timeinfo);
  std::string filename = buffer;
  
  std::cout << "Opening Logfile: "<< filename << "\n";
  logfile_.open ( filename.c_str() );
}

void leg_odometry::terminate(){
  std::cout << "Closing Logfile: " << "\n";
  logfile_.close();
}
  

void leg_odometry::initializePose(int mode,Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot){
  if (mode ==0){ 
    // Initialize with primary foot at zero
    world_to_fixed_primary_foot_ = Eigen::Isometry3d::Identity();
    world_to_body_ =world_to_fixed_primary_foot_*body_to_l_foot.inverse();
  }else if (mode ==1){
    // At the EST_ROBOT_STATE pose that was logged into the file
    world_to_body_ = world_to_body_bdi_;
    world_to_fixed_primary_foot_ = world_to_body_*body_to_l_foot;
  }
}
  
void leg_odometry::leg_odometry_basic(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status){
  if (!leg_odo_init_){
    if (contact_status == 2){
      std::cout << "Initialize Leg Odometry using left foot\n"; 
      initializePose(1, body_to_l_foot, body_to_r_foot);
      
      world_to_secondary_foot_ = world_to_body_*body_to_r_foot;
      primary_foot_ = 0; // left
      leg_odo_init_ = true;
    }
    // TODO: add ability to initialize off of right foot
  }else{
    if (contact_status == 2 && primary_foot_ ==0){
      if (verbose_>2) std::cout << "Using fixed Left foot, update pelvis position\n";
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_l_foot.inverse() ;
      world_to_secondary_foot_ = world_to_body_ * body_to_r_foot;
    }else if (contact_status == 1 && primary_foot_ == 0){
      std::cout << "Transition Odometry to right foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot
      world_to_fixed_primary_foot_ = world_to_secondary_foot_;
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_r_foot.inverse();
      world_to_secondary_foot_ = world_to_body_ * body_to_l_foot;
      primary_foot_ = 1;
    }else if (contact_status == 3 && primary_foot_ == 1){
      if (verbose_>2) std::cout << "Using fixed Right foot, update pelvis position\n";
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_r_foot.inverse() ;
      world_to_secondary_foot_ = world_to_body_ * body_to_l_foot;
    }else if (contact_status == 0 && primary_foot_ == 1){
      std::cout << "Transition Odometry to left foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot
      world_to_fixed_primary_foot_ = world_to_secondary_foot_;
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_l_foot.inverse();
      world_to_secondary_foot_ = world_to_body_ * body_to_r_foot;
      primary_foot_ = 0;
    }else{
      std::cout << "initialized but unknown update: " << contact_status << "\n";
    }
  }
}

void leg_odometry::leg_odometry_gravity_slaved_once(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status){
  if (!leg_odo_init_){
    if (contact_status == 2){
      std::cout << "Initialize Leg Odometry using left foot\n"; 
      initializePose(1, body_to_l_foot, body_to_r_foot);

      world_to_secondary_foot_ = world_to_body_*body_to_r_foot;
      primary_foot_ = 0; // left
      leg_odo_init_ = true;
    }
    // TODO: add ability to initialize off of right foot
  }else{
    if (contact_status == 2 && primary_foot_ ==0){
      if (verbose_>2) std::cout << "Using fixed Left foot, update pelvis position\n";
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_l_foot.inverse() ;
      world_to_secondary_foot_ = world_to_body_ * body_to_r_foot;
    }else if (contact_status == 1 && primary_foot_ == 0){
      std::cout << "Transition Odometry to right foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot
      
      // At the instant of transition, slave the pelvis position to gravity:
      // - retain the xyz position.
      // - take the pitch and roll from BDI
      // - take the yaw from previously
      // Then do FK and fix that position as the foot position
  
      Eigen::Quaterniond q_prev( world_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( world_to_body_bdi_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);
      Eigen::Isometry3d world_to_body_switch;
      world_to_body_switch.setIdentity();
      world_to_body_switch.translation() = world_to_body_.translation();
      world_to_body_switch.rotate(q_combined);
      world_to_fixed_primary_foot_ = world_to_body_switch * previous_body_to_r_foot_;
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_r_foot.inverse();
      world_to_secondary_foot_ = world_to_body_ * body_to_l_foot;
      primary_foot_ = 1;
    }else if (contact_status == 3 && primary_foot_ == 1){
      if (verbose_>2) std::cout << "Using fixed Right foot, update pelvis position\n";
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_r_foot.inverse() ;
      world_to_secondary_foot_ = world_to_body_ * body_to_l_foot;
    }else if (contact_status == 0 && primary_foot_ == 1){
      std::cout << "Transition Odometry to left foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot

      // At the instant of transition, slave the pelvis position to gravity:
      // - retain the xyz position.
      // - take the pitch and roll from BDI
      // - take the yaw from previously
      // Then do FK and fix that position as the foot position
  
      Eigen::Quaterniond q_prev( world_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( world_to_body_bdi_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);
      Eigen::Isometry3d world_to_body_switch;
      world_to_body_switch.setIdentity();
      world_to_body_switch.translation() = world_to_body_.translation();
      world_to_body_switch.rotate(q_combined);
      world_to_fixed_primary_foot_ = world_to_body_switch * previous_body_to_l_foot_;      
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_l_foot.inverse();
      world_to_secondary_foot_ = world_to_body_ * body_to_r_foot;
      primary_foot_ = 0;
    }else{
      std::cout << "initialized but unknown update: " << contact_status << "\n";
    }
  }
}

void leg_odometry::leg_odometry_gravity_slaved_always(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status){
  if (!leg_odo_init_){
    if (contact_status == 2){
      std::cout << "Initialize Leg Odometry using left foot\n"; 
      initializePose(1, body_to_l_foot, body_to_r_foot);
      
      world_to_secondary_foot_ = world_to_body_*body_to_r_foot;
      primary_foot_ = 0; // left
      leg_odo_init_ = true;
    }
    // TODO: add ability to initialize off of right foot
  }else{
    if (contact_status == 2 && primary_foot_ ==0){
      if (verbose_>2) std::cout << "Using fixed Left foot, update pelvis position\n";
      // Take the quaternion from BDI, apply the pitch and roll to the primary foot 
      // via fk. Then update the pelvis position
      
      Eigen::Quaterniond q_prev( world_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( world_to_body_bdi_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      // the slaved orientation of the pelvis
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);

      Eigen::Isometry3d world_to_body_at_zero;
      world_to_body_at_zero.setIdentity(); // ... Dont need to use the translation, so not filling it in
      world_to_body_at_zero.rotate(q_slaved);
      
      Eigen::Isometry3d world_to_r_foot_at_zero = world_to_body_at_zero*body_to_l_foot;
      Eigen::Quaterniond q_foot_new( world_to_r_foot_at_zero.rotation() );
      Eigen::Vector3d foot_new_trans = world_to_fixed_primary_foot_.translation();
      
      // the foot has now been rotated to agree with the pelvis orientation:
      world_to_fixed_primary_foot_.setIdentity();
      world_to_fixed_primary_foot_.translation() = foot_new_trans;
      world_to_fixed_primary_foot_.rotate( q_foot_new );      
      
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_l_foot.inverse() ;
      world_to_secondary_foot_ = world_to_body_ * body_to_r_foot;
    }else if (contact_status == 1 && primary_foot_ == 0){
      std::cout << "Transition Odometry to right foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot
      
      // At the instant of transition, slave the pelvis position to gravity:
      // - retain the xyz position.
      // - take the pitch and roll from BDI
      // - take the yaw from previously
      // Then do FK and fix that position as the foot position
  
      Eigen::Quaterniond q_prev( world_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( world_to_body_bdi_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);
      Eigen::Isometry3d world_to_body_switch;
      world_to_body_switch.setIdentity();
      world_to_body_switch.translation() = world_to_body_.translation();
      world_to_body_switch.rotate(q_slaved);
      world_to_fixed_primary_foot_ = world_to_body_switch * body_to_r_foot;
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_r_foot.inverse();
      world_to_secondary_foot_ = world_to_body_ * body_to_l_foot;
      primary_foot_ = 1;
    }else if (contact_status == 3 && primary_foot_ == 1){
      if (verbose_>2) std::cout << "Using fixed Right foot, update pelvis position\n";
      
      // Take the quaternion from BDI, apply the pitch and roll to the primary foot 
      // via fk. Then update the pelvis position
      Eigen::Quaterniond q_prev( world_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( world_to_body_bdi_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      // the slaved orientation of the pelvis
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);

      Eigen::Isometry3d world_to_body_at_zero;
      world_to_body_at_zero.setIdentity(); // ... Dont need to use the translation, so not filling it in
      world_to_body_at_zero.rotate(q_slaved);
      
      Eigen::Isometry3d world_to_r_foot_at_zero = world_to_body_at_zero*body_to_r_foot;
      Eigen::Quaterniond q_foot_new( world_to_r_foot_at_zero.rotation() );
      Eigen::Vector3d foot_new_trans = world_to_fixed_primary_foot_.translation();
      
      // the foot has now been rotated to agree with the pelvis orientation:
      world_to_fixed_primary_foot_.setIdentity();
      world_to_fixed_primary_foot_.translation() = foot_new_trans;
      world_to_fixed_primary_foot_.rotate( q_foot_new ); 
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_r_foot.inverse() ;
      world_to_secondary_foot_ = world_to_body_ * body_to_l_foot;
    }else if (contact_status == 0 && primary_foot_ == 1){
      std::cout << "Transition Odometry to left foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot

      // At the instant of transition, slave the pelvis position to gravity:
      // - retain the xyz position.
      // - take the pitch and roll from BDI
      // - take the yaw from previously
      // Then do FK and fix that position as the foot position
  
      Eigen::Quaterniond q_prev( world_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( world_to_body_bdi_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);
      Eigen::Isometry3d world_to_body_switch;
      world_to_body_switch.setIdentity();
      world_to_body_switch.translation() = world_to_body_.translation();
      world_to_body_switch.rotate(q_slaved);
      world_to_fixed_primary_foot_ = world_to_body_switch * body_to_l_foot;      
      
      world_to_body_ = world_to_fixed_primary_foot_ * body_to_l_foot.inverse();
      world_to_secondary_foot_ = world_to_body_ * body_to_r_foot;
      primary_foot_ = 0;
    }else{
      std::cout << "initialized but unknown update: " << contact_status << "\n";
    }
  }
}

void leg_odometry::Update(const  drc::robot_state_t* msg){

  
  // 0. Extract World Pose of body as estimated by BDI
  // primarily the orientation of of interest
  world_to_body_bdi_.setIdentity();
  world_to_body_bdi_.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body_bdi_.rotate(quat);    
    
  // 1. Solve for Forward Kinematics:
  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  map<string, KDL::Frame > cartpos_out;
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
  Eigen::Isometry3d body_to_l_foot = KDLToEigen(cartpos_out.find("l_foot")->second);
  Eigen::Isometry3d body_to_r_foot = KDLToEigen(cartpos_out.find("r_foot")->second);  

  // The Foot Contact Logic that Dehann wrote in the VRC:
  TwoLegs::footstep newstep;
  newstep = foot_contact_logic_->DetectFootTransistion(msg->utime, 
                                                       msg->force_torque.l_foot_force_z, 
                                                       msg->force_torque.r_foot_force_z);
  if (newstep.foot == LEFTFOOT || newstep.foot == RIGHTFOOT) {
    foot_contact_logic_->setStandingFoot(newstep.foot);
  }
  
  
  int contact_status_new;  
  if (newstep.foot != -1){
    std::cout << "NEW STEP ON " << ((foot_contact_logic_->secondary_foot()==LEFTFOOT) ? "LEFT" : "RIGHT")  << std::endl;
    if ( foot_contact_logic_->getStandingFoot() == LEFTFOOT ){
      contact_status_new = 0;
    }else if ( foot_contact_logic_->getStandingFoot() == RIGHTFOOT ){
      contact_status_new = 1;
    }else{
      std::cout << "Foot Contact Error "<< foot_contact_logic_->getStandingFoot() << " (switch)\n";
      int blah;
      cin >> blah;      
    }    
  }else{
    if ( foot_contact_logic_->getStandingFoot() == LEFTFOOT ){
      contact_status_new = 2;
    }else if ( foot_contact_logic_->getStandingFoot() == RIGHTFOOT ){
      contact_status_new = 3;
    }else{
      std::cout << "NEW Foot Contact Error "<< foot_contact_logic_->getStandingFoot() << " \n";
      int blah;
      cin >> blah;      
    }
  }
  
  
  int mode = 0;
  if (leg_odometry_mode_ == "basic" ){
    leg_odometry_basic(body_to_l_foot, body_to_r_foot, contact_status_new);
  }else if (leg_odometry_mode_ == "slaved_once" ){  
    leg_odometry_gravity_slaved_once(body_to_l_foot, body_to_r_foot, contact_status_new);
  }else if( leg_odometry_mode_ == "slaved_always" ){  
    leg_odometry_gravity_slaved_always(body_to_l_foot, body_to_r_foot, contact_status_new);
  }else{
    std::cout << "Unrecognised odometry algorithm\n"; 
    exit(-1);
  }
    
  
  
  
  if (leg_odo_init_){
    std::vector<Isometry3dTime> world_to_body_T;
    world_to_body_T.push_back( Isometry3dTime(msg->utime , world_to_body_  )  );
    pc_vis_->pose_collection_to_lcm_from_list(1001, world_to_body_T);
    
    std::vector<Isometry3dTime> world_to_primary_T;
    world_to_primary_T.push_back( Isometry3dTime(msg->utime , world_to_fixed_primary_foot_  )  );
    pc_vis_->pose_collection_to_lcm_from_list(1002, world_to_primary_T);

    std::vector<Isometry3dTime> world_to_secondary_T;
    world_to_secondary_T.push_back( Isometry3dTime(msg->utime , world_to_secondary_foot_  )  );
    pc_vis_->pose_collection_to_lcm_from_list(1003, world_to_secondary_T);
    
    // BDI estimated:
    if (cl_cfg_.read_lcmlog){
      bot_core::pose_t pose_msg = getPoseAsBotPose(world_to_body_bdi_, msg->utime);
      lcm_publish_->publish("POSE_BODY", &pose_msg );
      lcm_publish_->publish("EST_ROBOT_STATE", msg);
    }

    
    // MIT estimated:
    bot_core::pose_t pose_msg = getPoseAsBotPose(world_to_body_, msg->utime);
    lcm_publish_->publish("POSE_BODY_ALT", &pose_msg );    
    drc::robot_state_t msg_out = *msg;
    insertPoseInRobotState(msg_out, world_to_body_);
    lcm_publish_->publish("EST_ROBOT_STATE_COMPRESSED_LOOPBACK", &msg_out );
    
    std::stringstream ss;
    ss << print_Isometry3d(world_to_body_bdi_) << ", "
       << print_Isometry3d(world_to_body_);
       
    // std::cout << ss.str() << "\n";
    logfile_ << ss.str() << "\n";
  }
  
  previous_body_to_l_foot_ = body_to_l_foot;
  previous_body_to_r_foot_ = body_to_r_foot;
}