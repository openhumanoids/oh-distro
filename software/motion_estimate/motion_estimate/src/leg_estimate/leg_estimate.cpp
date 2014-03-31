#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>
#include <fstream>

#include "leg_estimate.hpp"



using namespace std;
using namespace boost;
using namespace boost::assign;

leg_estimate::leg_estimate( boost::shared_ptr<lcm::LCM> &lcm_publish_,
  BotParam * botparam_, boost::shared_ptr<ModelClient> &model_):
  lcm_publish_(lcm_publish_),  botparam_(botparam_), model_(model_),
  lfoot_sensing_(0,0,0), rfoot_sensing_(0,0,0) {
  
  initialization_mode_ = bot_param_get_str_or_fail(botparam_, "state_estimator.legodo.initialization_mode");
  std::cout << "Leg Odometry Initialize Mode: " << initialization_mode_ << " \n";
  leg_odometry_mode_ = bot_param_get_str_or_fail(botparam_, "state_estimator.legodo.integration_mode");
  std::cout << "Leg Odometry Accumulation Mode: " << leg_odometry_mode_ << " \n";
  
  string standing_link= bot_param_get_str_or_fail(botparam_, "state_estimator.legodo.standing_link");
  std::cout << "Leg Odometry Standing Link: " << standing_link << " \n";
  l_standing_link_ = "l_" + standing_link;
  r_standing_link_ = "r_" + standing_link;
  
  filter_joint_positions_ = bot_param_get_boolean_or_fail(botparam_, "state_estimator.legodo.filter_joint_positions");
  std::cout << "Leg Odometry Filter Joints: " << filter_joint_positions_ << " \n";
  
  filter_contact_events_ = bot_param_get_boolean_or_fail(botparam_, "state_estimator.legodo.filter_contact_events");
  std::cout << "Leg Odometry Filter Contact Events: " << filter_contact_events_ << " \n";
  
  publish_diagnostics_ = bot_param_get_boolean_or_fail(botparam_, "state_estimator.legodo.publish_diagnostics");  
  
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_publish_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1001,"Body Pose [odom]",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1002,"Primary Foot [odom] ",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1004,"Primary Contacts [odom] ",1,0, 1002,1, { 0.0, 1.0, 0.0} ));  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1003,"Secondary Foot [odom] ",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1005,"Secondary Contacts [odom] ",1,0, 1003,1, { 1.0, 0.0, 0.0} ));  
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1011,"Body Pose [world]",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1012,"Primary Foot [world] ",5,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1013,"Secondary Foot [world] ",5,1) );

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1014,"Primary Foot transition [world] ",5,0) );
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1021,"Body Pose [const]",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1022,"Primary Foot [const] ",5,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1023,"Secondary Foot [const] ",5,1) );
  
  
  // actually more like 1540N when standing still in Jan 2014
  float atlas_weight = 1400.0;
  // originally foot shift was when s_foot - 1400*0.65  > p_foot   ... typically s_foot = 1180 | p_foot =200 (~75%)
  foot_contact_logic_ = new TwoLegs::FootContact(false, atlas_weight);
  foot_contact_logic_->setStandingFoot( FOOT_LEFT );

  foot_contact_logic_alt_ = new TwoLegs::FootContactAlt(false, atlas_weight);
  foot_contact_logic_alt_->setStandingFoot( F_LEFT );
  
  // these two variables are probably duplicate - need to clean this up...
  primary_foot_ = F_LEFT; // ie left
  standing_foot_ = 0; // 
  leg_odo_init_ = false;
   
  foot_contact_classify_ = new foot_contact_classify(lcm_publish_,publish_diagnostics_);

  verbose_ = 1;
  
  previous_utime_ = 0; // Set utimes to known values
  current_utime_ = 0;
  
  odom_to_body_.setIdentity();
  bdi_to_body_.setIdentity();
  world_to_body_.setIdentity();
  world_to_body_init_ = false;
  world_to_primary_foot_transition_init_ = false;
  world_to_body_constraint_init_ = false;
}

// Very Basic switch to determine FK of primary and secondary foot
// By providing the id, I can use this flexiably
// TODO: use this in the integration do reduce all the cases
Eigen::Isometry3d getPrimaryFootFK(footid_alt main_id, Eigen::Isometry3d body_to_l_foot, Eigen::Isometry3d body_to_r_foot){
  if (main_id == F_LEFT){
    return body_to_l_foot;
  }else if(main_id == F_RIGHT){
    return body_to_r_foot;
  }else{
    std::cout << "ERROR: foot id out of range! getPrimaryFootFK\n";
    return Eigen::Isometry3d::Identity();
  }
}

Eigen::Isometry3d getSecondaryFootFK(footid_alt main_id, Eigen::Isometry3d body_to_l_foot, Eigen::Isometry3d body_to_r_foot){
  if (main_id == F_RIGHT){
    return body_to_l_foot;
  }else if(main_id == F_LEFT){
    return body_to_r_foot;
  }else{
    std::cout << "ERROR: foot id out of range! getSecondaryFootFK\n";
    return Eigen::Isometry3d::Identity();
  }
}


// TODO: need to move this function outside of the class, down to app
bool leg_estimate::initializePose(Eigen::Isometry3d body_to_foot){
  if (initialization_mode_ == "zero"){ 
    // Initialize with primary foot at (0,0,0) but orientation using bdi rotation:
    // Otherwise, there is a discontinuity at the very start
    Eigen::Quaterniond q_slaved( bdi_to_body_.rotation() );
    Eigen::Isometry3d odom_to_body_at_zero;
    odom_to_body_at_zero.setIdentity(); // ... Dont need to use the translation, so not filling it in
    odom_to_body_at_zero.rotate(q_slaved);
    Eigen::Isometry3d odom_to_r_foot_at_zero = odom_to_body_at_zero*body_to_foot;
    Eigen::Quaterniond q_foot_new( odom_to_r_foot_at_zero.rotation() );
    odom_to_primary_foot_fixed_ = Eigen::Isometry3d::Identity();
    odom_to_primary_foot_fixed_.rotate( q_foot_new );     
    
    // was...
    //odom_to_primary_foot_fixed_ = Eigen::Isometry3d::Identity();
    odom_to_body_ =odom_to_primary_foot_fixed_*body_to_foot.inverse();
  }else if (initialization_mode_ == "bdi"){
    // At the EST_ROBOT_STATE pose that was logged into the file
    odom_to_body_ = bdi_to_body_;
    odom_to_primary_foot_fixed_ = odom_to_body_*body_to_foot;
  }
  return true;
}

bool leg_estimate::prepInitialization(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status){
  bool init_this_iteration = false;
  if (contact_status == F_LEFT_FIXED){
    std::cout << "Initialize Leg Odometry using left foot\n"; 
    bool success = initializePose(body_to_l_foot); // typical init mode =0
    if (success){
      // if successful, complete initialization
      primary_foot_ = F_LEFT; // left
      odom_to_secondary_foot_ = odom_to_body_*body_to_r_foot;
      leg_odo_init_ = true;
      init_this_iteration = true;
    }
  }else if  (contact_status == F_RIGHT_FIXED){
    std::cout << "Initialize Leg Odometry using left foot\n"; 
    bool success = initializePose(body_to_r_foot); // typical init mode =0
    if (success){
      // if successful, complete initialization
      primary_foot_ = F_RIGHT; // right
      odom_to_secondary_foot_ = odom_to_body_*body_to_l_foot;
      leg_odo_init_ = true;
      init_this_iteration = true;
    }      
  }  
  
  return init_this_iteration;
}

  
bool leg_estimate::leg_odometry_basic(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status){
  bool init_this_iteration= false;

  if (!leg_odo_init_){
    init_this_iteration = prepInitialization(body_to_l_foot, body_to_r_foot, contact_status);
  }else{
    if (contact_status == F_LEFT_FIXED && primary_foot_ == F_LEFT){
      if (verbose_>2) std::cout << "Using fixed Left foot, update pelvis position\n";
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse() ;
      odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
    }else if (contact_status == F_RIGHT_NEW && primary_foot_ == F_LEFT){
      std::cout << "Transition Odometry to right foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot
      odom_to_primary_foot_fixed_ = odom_to_secondary_foot_;
      
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse();
      odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
      primary_foot_ = F_RIGHT;
    }else if (contact_status == F_RIGHT_FIXED && primary_foot_ == F_RIGHT){
      if (verbose_>2) std::cout << "Using fixed Right foot, update pelvis position\n";
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse() ;
      odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
    }else if (contact_status == F_LEFT_NEW && primary_foot_ == F_RIGHT){
      std::cout << "Transition Odometry to left foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot
      odom_to_primary_foot_fixed_ = odom_to_secondary_foot_;
      
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse();
      odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
      primary_foot_ = F_LEFT;
    }else{
      std::cout << "initialized but unknown update: " << contact_status << " and " << (int) primary_foot_ << "\n";
    }
    
  }
  
  return init_this_iteration;
}

bool leg_estimate::leg_odometry_gravity_slaved_once(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status){
  bool init_this_iteration= false;
  
  if (!leg_odo_init_){
    init_this_iteration = prepInitialization(body_to_l_foot, body_to_r_foot, contact_status);
  }else{
    if (contact_status == F_LEFT_FIXED && primary_foot_ == F_LEFT){
      if (verbose_>2) std::cout << "Using fixed Left foot, update pelvis position\n";
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse() ;
      odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
    }else if (contact_status == F_RIGHT_NEW && primary_foot_ == F_LEFT){
      std::cout << "Transition Odometry to right foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot
      
      // At the instant of transition, slave the pelvis position to gravity:
      // - retain the xyz position.
      // - take the pitch and roll from BDI
      // - take the yaw from previously
      // Then do FK and fix that position as the foot position
  
      Eigen::Quaterniond q_prev( odom_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( bdi_to_body_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);
      Eigen::Isometry3d odom_to_body_switch;
      odom_to_body_switch.setIdentity();
      odom_to_body_switch.translation() = odom_to_body_.translation();
      odom_to_body_switch.rotate(q_combined);
      odom_to_primary_foot_fixed_ = odom_to_body_switch * previous_body_to_r_foot_;
      
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse();
      odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
      primary_foot_ = F_RIGHT;
    }else if (contact_status == F_RIGHT_FIXED && primary_foot_ == F_RIGHT){
      if (verbose_>2) std::cout << "Using fixed Right foot, update pelvis position\n";
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse() ;
      odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
    }else if (contact_status == F_LEFT_NEW && primary_foot_ == F_RIGHT){
      std::cout << "Transition Odometry to left foot. Fix foot, update pelvis position\n";
      // When transitioning, take the passive position of the other foot
      // from the previous iteration. this will now be the fixed foot

      // At the instant of transition, slave the pelvis position to gravity:
      // - retain the xyz position.
      // - take the pitch and roll from BDI
      // - take the yaw from previously
      // Then do FK and fix that position as the foot position
  
      Eigen::Quaterniond q_prev( odom_to_body_.rotation() );
      double rpy_prev[3];
      quat_to_euler(q_prev, rpy_prev[0],rpy_prev[1],rpy_prev[2]);

      Eigen::Quaterniond q_slaved( bdi_to_body_.rotation() );
      double rpy_slaved[3];
      quat_to_euler(q_slaved, rpy_slaved[0],rpy_slaved[1],rpy_slaved[2]);
      
      Eigen::Quaterniond q_combined = euler_to_quat( rpy_slaved[0], rpy_slaved[1], rpy_prev[2]);
      Eigen::Isometry3d odom_to_body_switch;
      odom_to_body_switch.setIdentity();
      odom_to_body_switch.translation() = odom_to_body_.translation();
      odom_to_body_switch.rotate(q_combined);
      odom_to_primary_foot_fixed_ = odom_to_body_switch * previous_body_to_l_foot_;      
      
      odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse();
      odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
      primary_foot_ = F_LEFT;
    }else{
      std::cout << "initialized but unknown update: " << contact_status << " and " << (int) primary_foot_ << "\n";
    }
  }
  
  return init_this_iteration;
}

bool leg_estimate::leg_odometry_gravity_slaved_always(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status){
  bool init_this_iteration= false;
  
  if (!leg_odo_init_){
    init_this_iteration = prepInitialization(body_to_l_foot, body_to_r_foot, contact_status);
    return init_this_iteration;
  }
    
  if (contact_status == F_LEFT_FIXED && primary_foot_ == F_LEFT){
    if (verbose_>2) std::cout << "Using fixed Left foot, update pelvis position\n";

    // Apply the quaternion from BDI to the primary foot 
    // via fk. Then update the pelvis position
    Eigen::Isometry3d odom_to_body_at_zero = Eigen::Isometry3d::Identity(); // ... Dont need to use the translation, so not filling it in
    odom_to_body_at_zero.rotate( Eigen::Quaterniond(bdi_to_body_.rotation()) );
    
    Eigen::Isometry3d odom_to_r_foot_at_zero = odom_to_body_at_zero*body_to_l_foot;
    Eigen::Quaterniond q_foot_new( odom_to_r_foot_at_zero.rotation() );
    Eigen::Vector3d foot_new_trans = odom_to_primary_foot_fixed_.translation();
    
    // the foot has now been rotated to agree with the pelvis orientation:
    odom_to_primary_foot_fixed_.setIdentity();
    odom_to_primary_foot_fixed_.translation() = foot_new_trans;
    odom_to_primary_foot_fixed_.rotate( q_foot_new );      
    
    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse() ;
    odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
  }else if (contact_status == F_RIGHT_NEW && primary_foot_ == F_LEFT){
    std::cout << "Transition Odometry to right foot. Fix foot, update pelvis position\n";
    // When transitioning, take the passive position of the other foot
    // from the previous iteration. this will now be the fixed foot
    // At the instant of transition, slave the pelvis position to gravity:
    // - retain the xyz position.
    // - take the orientation from BDI
    // Then do FK and fix that position as the foot position
    Eigen::Isometry3d odom_to_body_switch = Eigen::Isometry3d::Identity();
    odom_to_body_switch.translation() = odom_to_body_.translation();
    odom_to_body_switch.rotate( Eigen::Quaterniond(bdi_to_body_.rotation()) );
    odom_to_primary_foot_fixed_ = odom_to_body_switch * body_to_r_foot;
    
    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse();
    odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
    primary_foot_ = F_RIGHT;
  }else if (contact_status == F_RIGHT_FIXED && primary_foot_ == F_RIGHT){
    if (verbose_>2) std::cout << "Using fixed Right foot, update pelvis position\n";
    
    // Apply the quaternion from BDI to the primary foot 
    // via fk. Then update the pelvis position
    Eigen::Isometry3d odom_to_body_at_zero = Eigen::Isometry3d::Identity(); // ... Dont need to use the translation, so not filling it in
    odom_to_body_at_zero.rotate( Eigen::Quaterniond(bdi_to_body_.rotation()) );
    
    Eigen::Isometry3d odom_to_r_foot_at_zero = odom_to_body_at_zero*body_to_r_foot;
    Eigen::Quaterniond q_foot_new( odom_to_r_foot_at_zero.rotation() );
    Eigen::Vector3d foot_new_trans = odom_to_primary_foot_fixed_.translation();
    
    // the foot has now been rotated to agree with the pelvis orientation:
    odom_to_primary_foot_fixed_.setIdentity();
    odom_to_primary_foot_fixed_.translation() = foot_new_trans;
    odom_to_primary_foot_fixed_.rotate( q_foot_new ); 
    
    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_r_foot.inverse() ;
    odom_to_secondary_foot_ = odom_to_body_ * body_to_l_foot;
  }else if (contact_status == F_LEFT_NEW && primary_foot_ == F_RIGHT){
    std::cout << "Transition Odometry to left foot. Fix foot, update pelvis position\n";
    
    // When transitioning, take the passive position of the other foot
    // from the previous iteration. this will now be the fixed foot
    // At the instant of transition, slave the pelvis position to gravity:
    // - retain the xyz position.
    // - take the orientation from BDI
    // Then do FK and fix that position as the foot position
    Eigen::Isometry3d odom_to_body_switch = Eigen::Isometry3d::Identity();
    odom_to_body_switch.translation() = odom_to_body_.translation();
    odom_to_body_switch.rotate( Eigen::Quaterniond(bdi_to_body_.rotation()) );
    odom_to_primary_foot_fixed_ = odom_to_body_switch * body_to_l_foot;      
    
    odom_to_body_ = odom_to_primary_foot_fixed_ * body_to_l_foot.inverse();
    odom_to_secondary_foot_ = odom_to_body_ * body_to_r_foot;
    primary_foot_ = F_LEFT;
  }else{
    std::cout << "initialized but unknown update: " << contact_status << " and " << (int) primary_foot_ << "\n";
  }
    
  return init_this_iteration;
}



void leg_estimate::determine_position_constraint_slaved_always(Eigen::Isometry3d body_to_l_foot, Eigen::Isometry3d body_to_r_foot){
  // Take the CURRENT quaternion from [[[POSE_BODY]]] - via FK.
  // Then update the pelvis position
  Eigen::Isometry3d world_to_body_at_zero = Eigen::Isometry3d::Identity(); // ... Dont need to use the translation, so not filling it in
  world_to_body_at_zero.rotate( Eigen::Quaterniond(world_to_body_.rotation()) );
  
  Eigen::Isometry3d world_to_primary_at_zero = world_to_body_at_zero* getPrimaryFootFK(primary_foot_,body_to_l_foot, body_to_r_foot) ;
  Eigen::Quaterniond q_foot_new( world_to_primary_at_zero.rotation() ); // assumed to be current foot orientation (in world)
  Eigen::Vector3d foot_new_trans = world_to_primary_foot_transition_.translation();
  
  // the foot has now been rotated to agree with the pelvis orientation and translated to express the constraint
  world_to_primary_foot_constraint_.setIdentity();
  world_to_primary_foot_constraint_.translation() = foot_new_trans;
  world_to_primary_foot_constraint_.rotate( q_foot_new ); 
  
  world_to_body_constraint_ = world_to_primary_foot_constraint_ * getPrimaryFootFK(primary_foot_,body_to_l_foot, body_to_r_foot).inverse() ;
  world_to_secondary_foot_constraint_ = world_to_body_constraint_ * getSecondaryFootFK(primary_foot_,body_to_l_foot, body_to_r_foot);
  world_to_body_constraint_init_ = true;
}


contact_status_id leg_estimate::footTransition(){
  contact_status_id contact_status = F_STATUS_UNKNOWN;  
  
  //std::cout << lfoot_sensing_.force_z <<  " | " << rfoot_sensing_.force_z << "\n";
  footid newstep = foot_contact_logic_->DetectFootTransition(current_utime_, lfoot_sensing_.force_z, rfoot_sensing_.force_z);
  if (newstep == FOOT_LEFT || newstep == FOOT_RIGHT) {
    foot_contact_logic_->setStandingFoot(newstep);
  }
  if (newstep != FOOT_UNKNOWN){
    std::cout << "NEW STEP | STANDING ON " << ((foot_contact_logic_->getStandingFoot()==FOOT_LEFT) ? "LEFT" : "RIGHT")  << std::endl;
    if ( foot_contact_logic_->getStandingFoot() == FOOT_LEFT ){
      contact_status = F_LEFT_NEW;
    }else if ( foot_contact_logic_->getStandingFoot() == FOOT_RIGHT ){
      contact_status = F_RIGHT_NEW;
    }else{
      std::cout << "Foot Contact Error "<< foot_contact_logic_->getStandingFoot() << " (switch)\n";
      int blah;
      cin >> blah;      
    }    
  }else{
    if ( foot_contact_logic_->getStandingFoot() == FOOT_LEFT ){
      contact_status = F_LEFT_FIXED;
    }else if ( foot_contact_logic_->getStandingFoot() == FOOT_RIGHT ){
      contact_status = F_RIGHT_FIXED;
    }else{
      std::cout << "Foot Contact Error "<< foot_contact_logic_->getStandingFoot() << " \n";
      int blah;
      cin >> blah;      
    }
  }  
 
  standing_foot_ = foot_contact_logic_->getStandingFoot();
 
  return contact_status;
}


contact_status_id leg_estimate::footTransitionAlt(){
  contact_status_id contact_status = foot_contact_logic_alt_->DetectFootTransition(current_utime_, lfoot_sensing_.force_z, rfoot_sensing_.force_z);
  
  standing_foot_ = foot_contact_logic_alt_->getStandingFoot();
  return contact_status;
}




float leg_estimate::updateOdometry(std::vector<std::string> joint_name, std::vector<float> joint_position, int64_t utime){
  previous_utime_ = current_utime_;
  previous_odom_to_body_ = odom_to_body_;
  current_utime_ = utime;
  
  if ( (current_utime_ - previous_utime_)*1E-6 > 30E-3){
    double odo_dt = (current_utime_ - previous_utime_)*1E-6;
    std::cout << "extended time since last update: " <<  odo_dt << "\n";
    std::cout << "resetting the leg odometry\n";
    leg_odo_init_ = false;
  }
  
  
  // 0. Filter Joints
  if (filter_joint_positions_){
    double scale = 1/1.0091; // TODO: correct none-unity coeff sum at source: (a bug in dehann's code)
    for (size_t i=0 ; i < 28; i++){
      joint_position[i]  = scale*lpfilter_[i].processSample( joint_position[i] );
    }
  }  

  // 1. Solve for Forward Kinematics:
  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  map<string, KDL::Frame > cartpos_out;
  for (size_t i=0; i<  joint_name.size(); i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(joint_name[i], joint_position[i]));
  bool kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_out,true); // true = flatten tree to absolute transforms
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    exit(-1);
  }
  Eigen::Isometry3d body_to_l_foot = KDLToEigen(cartpos_out.find(l_standing_link_)->second);
  Eigen::Isometry3d body_to_r_foot = KDLToEigen(cartpos_out.find(r_standing_link_)->second);  

  // 2. Determine Primary Foot State
  
  // 5. Analyse signals to infer covariance
  // Classify/Detect Contact events: strike, break, swing, sway
  foot_contact_classify_->setFootSensing(lfoot_sensing_, rfoot_sensing_);
  float contact_classification = foot_contact_classify_->update(current_utime_, odom_to_primary_foot_fixed_, 
                                                       odom_to_secondary_foot_, standing_foot_);  
  contact_status_id contact_status = F_STATUS_UNKNOWN;
  if (1==0){
    contact_status =footTransition(); // original method from Dehann
  }else{
    contact_status =  footTransitionAlt();
  }
  
  // 3. Integrate the Leg Kinematics
  bool init_this_iteration = false;
  if (leg_odometry_mode_ == "basic" ){
    init_this_iteration = leg_odometry_basic(body_to_l_foot, body_to_r_foot, contact_status);
  }else if (leg_odometry_mode_ == "slaved_once" ){  
    init_this_iteration = leg_odometry_gravity_slaved_once(body_to_l_foot, body_to_r_foot, contact_status);
  }else if( leg_odometry_mode_ == "slaved_always" ){  
    init_this_iteration = leg_odometry_gravity_slaved_always(body_to_l_foot, body_to_r_foot, contact_status);
  }else{
    std::cout << "Unrecognised odometry algorithm\n"; 
    exit(-1);
  }
    
  // NB: the corresponding variables are sent
  if (world_to_body_init_){
    world_to_primary_foot_slide_ = world_to_body_*getPrimaryFootFK(primary_foot_, body_to_l_foot, body_to_r_foot);    
    world_to_secondary_foot_ = world_to_body_*getSecondaryFootFK(primary_foot_, body_to_l_foot, body_to_r_foot);  
    if ( (contact_status == F_LEFT_NEW) || (contact_status == F_RIGHT_NEW) ){
      std::cout << "Leg Estimate: Changing Foot Position Constraint\n";
      world_to_primary_foot_transition_ = world_to_primary_foot_slide_;
      world_to_primary_foot_transition_init_ = true;
      Isometry3dTime world_to_primary_trans_T = Isometry3dTime(current_utime_ , world_to_primary_foot_transition_ ) ;
      pc_vis_->pose_to_lcm_from_list(1014, world_to_primary_trans_T);
    }
  }  
  
  
  
    
  // 4. Determine a valid kinematic delta
  float estimate_status = -1.0; // if odometry is not valid, then use -1 to indicate it
  if (leg_odo_init_){
    if (!init_this_iteration){
      // Calculate and publish the position delta:
      odom_to_body_delta_ =  previous_odom_to_body_.inverse() * odom_to_body_; 
      estimate_status = 0.0; // assume very accurate to begin with
      
      if (world_to_body_init_ && world_to_primary_foot_transition_init_){
        determine_position_constraint_slaved_always(body_to_l_foot, body_to_r_foot);
      }else{
        world_to_body_constraint_init_ = false;
      }
      
      
      if (publish_diagnostics_){
        Eigen::Vector3d motion_T = odom_to_body_delta_.translation();
        Eigen::Quaterniond motion_R = Eigen::Quaterniond(odom_to_body_delta_.rotation());
        drc::pose_transform_t legodo_msg;
        legodo_msg.utime = current_utime_;
        legodo_msg.prev_utime = previous_utime_;
        legodo_msg.translation[0] = motion_T(0);
        legodo_msg.translation[1] = motion_T(1);
        legodo_msg.translation[2] = motion_T(2);
        legodo_msg.rotation[0] = motion_R.w();
        legodo_msg.rotation[1] = motion_R.x();
        legodo_msg.rotation[2] = motion_R.y();
        legodo_msg.rotation[3] = motion_R.z();    
        lcm_publish_->publish("LEG_ODOMETRY_DELTA", &legodo_msg); // Outputting this message should enable out of core integration
      }
    }

    
    if (publish_diagnostics_){
      // TODO: pass Isometry3dTime by reference
      Isometry3dTime isoT = Isometry3dTime(current_utime_ , Eigen::Isometry3d::Identity() );
      isoT= Isometry3dTime(current_utime_ , odom_to_body_ ) ;
      pc_vis_->pose_to_lcm_from_list(1001, isoT);
      isoT = Isometry3dTime(current_utime_ , odom_to_primary_foot_fixed_ ) ;
      pc_vis_->pose_to_lcm_from_list(1002, isoT);
      isoT = Isometry3dTime(current_utime_ , odom_to_secondary_foot_ ) ;
      pc_vis_->pose_to_lcm_from_list(1003, isoT);
      // Primary (green) and Secondary (red) Contact points:
      pc_vis_->ptcld_to_lcm_from_list(1004, *foot_contact_classify_->getContactPoints() , current_utime_, current_utime_);
      pc_vis_->ptcld_to_lcm_from_list(1005, *foot_contact_classify_->getContactPoints() , current_utime_, current_utime_);
      
      if (world_to_body_init_ && world_to_primary_foot_transition_init_){
        isoT = Isometry3dTime(current_utime_ , world_to_body_ ) ;
        pc_vis_->pose_to_lcm_from_list(1011, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_primary_foot_slide_ ) ;
        pc_vis_->pose_to_lcm_from_list(1012, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_secondary_foot_ ) ;
        pc_vis_->pose_to_lcm_from_list(1013, isoT);
      
        isoT = Isometry3dTime(current_utime_ , world_to_body_constraint_ ) ;
        pc_vis_->pose_to_lcm_from_list(1021, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_primary_foot_constraint_ ) ;
        pc_vis_->pose_to_lcm_from_list(1022, isoT);
        isoT = Isometry3dTime(current_utime_ , world_to_secondary_foot_constraint_ ) ;
        pc_vis_->pose_to_lcm_from_list(1023, isoT);
      }
    }
  }
 
  
  if (filter_contact_events_){
    if (estimate_status > -1){ 
      // if the estimator reports a suitable estimate, use the classifier
      // Skip integration of odometry deemed to be unsuitable
      estimate_status = contact_classification;
    }
  }
  
  previous_body_to_l_foot_ = body_to_l_foot;
  previous_body_to_r_foot_ = body_to_r_foot;
  return estimate_status;
}
