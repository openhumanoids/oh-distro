#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>

#include "leg_odometry.hpp"
#include <ConciseArgs>

using namespace std;
using namespace boost;
using namespace boost::assign;

leg_odometry::leg_odometry(boost::shared_ptr<lcm::LCM> &lcm_):
          lcm_(lcm_){
            
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
            
            
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
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1001,"Body Pose",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1002,"Primary Foot",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1003,"Secondary Foot",5,1) );
  
  lcm_->subscribe("EST_ROBOT_STATE",&leg_odometry::robot_state_handler,this);  
  lcm_->subscribe("FOOT_CONTACT_ESTIMATE",&leg_odometry::foot_contact_handler,this);
  
  foot_contact_ = new foot_contact();
  last_left_contact_ = false;
  last_right_contact_ = false;
}

Eigen::Isometry3d KDLToEigen(KDL::Frame tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion( q.x() , q.y(), q.z(), q.w());
  tf_out.rotate(q);    
  return tf_out;
}

void leg_odometry::publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();  
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();  
  pose_msg.orientation[1] =  r_x.x();  
  pose_msg.orientation[2] =  r_x.y();  
  pose_msg.orientation[3] =  r_x.z();  
  lcm_->publish( channel, &pose_msg);
}




void leg_odometry::leg_odometry_basic(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status){
  if (!leg_odo_init_){
    if (contact_status == 2){
      std::cout << "Initialize Leg Odometry using left foot\n"; 
      // Initialize with primary foot at zero
      world_to_fixed_primary_foot_ = Eigen::Isometry3d::Identity();
      world_to_body_ =world_to_fixed_primary_foot_*body_to_l_foot.inverse();
      world_to_secondary_foot_ = world_to_body_*body_to_r_foot;
      primary_foot_ = 0; // left
      leg_odo_init_ = true;
    }
    // TODO: add ability to initialize off of right foot
  }else{
    if (contact_status == 2 && primary_foot_ ==0){
      std::cout << "Using fixed Left foot, update pelvis position\n";
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
      std::cout << "Using fixed Right foot, update pelvis position\n";
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

void leg_odometry::leg_odometry_gravity_slaved(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status){
  if (!leg_odo_init_){
    if (contact_status == 2){
      std::cout << "Initialize Leg Odometry using left foot\n"; 
      // Initialize with primary foot at zero
      world_to_fixed_primary_foot_ = Eigen::Isometry3d::Identity();
      world_to_body_ =world_to_fixed_primary_foot_*body_to_l_foot.inverse();
      world_to_secondary_foot_ = world_to_body_*body_to_r_foot;
      primary_foot_ = 0; // left
      leg_odo_init_ = true;
    }
    // TODO: add ability to initialize off of right foot
  }else{
    if (contact_status == 2 && primary_foot_ ==0){
      std::cout << "Using fixed Left foot, update pelvis position\n";
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
      std::cout << "Using fixed Right foot, update pelvis position\n";
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

void leg_odometry::leg_odometry_continually_gravity_slaved(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status){
  if (!leg_odo_init_){
    if (contact_status == 2){
      std::cout << "Initialize Leg Odometry using left foot\n"; 
      // Initialize with primary foot at zero
      world_to_fixed_primary_foot_ = Eigen::Isometry3d::Identity();
      world_to_body_ =world_to_fixed_primary_foot_*body_to_l_foot.inverse();
      world_to_secondary_foot_ = world_to_body_*body_to_r_foot;
      primary_foot_ = 0; // left
      leg_odo_init_ = true;
    }
    // TODO: add ability to initialize off of right foot
  }else{
    if (contact_status == 2 && primary_foot_ ==0){
      std::cout << "Using fixed Left foot, update pelvis position\n";
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
      std::cout << "Using fixed Right foot, update pelvis position\n";
      
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



void leg_odometry::foot_contact_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::foot_contact_estimate_t* msg){
  // These measurements are cached as the foot_contact class has state and requires
  // a continous sequence to ensure the state machine isnt broken
  // a better solution would be to have the foot contact estimator
  // be calculated in this process
  last_left_contact_ = msg->left_contact;
  last_right_contact_ = msg->right_contact;
}

void leg_odometry::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  // std::cout << "got foot con: "<<msg->left_contact << " and " << msg->right_contact <<"\n";
  
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

  
  int contact_status = foot_contact_->update(last_left_contact_, last_right_contact_);
  if (contact_status < 0){
    std::cout << "Feet not in contact yet... not integrating\n";  
  }
  

  int mode = 2;
  if (mode==0){
    leg_odometry_basic(body_to_l_foot, body_to_r_foot, contact_status);
  }else if (mode==1){  
    leg_odometry_gravity_slaved(body_to_l_foot, body_to_r_foot, contact_status);
  }else{  
    leg_odometry_continually_gravity_slaved(body_to_l_foot, body_to_r_foot, contact_status);
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
    
    publishPose(world_to_body_, msg->utime, "POSE_BODY_ALT");
  }
  
  previous_body_to_l_foot_ = body_to_l_foot;
  previous_body_to_r_foot_ = body_to_r_foot;
}


// Visualize the foot positions from BDI:
// This can be turnned off if necessary - its not important
void leg_odometry::foot_pos_est_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_foot_pos_est_t* msg){
  Eigen::Isometry3d left_pos;
  left_pos.setIdentity();
  left_pos.translation()  << msg->left_position[0], msg->left_position[1], msg->left_position[2];
      
  Eigen::Isometry3d right_pos;
  right_pos.setIdentity();
  right_pos.translation()  << msg->right_position[0], msg->right_position[1], msg->right_position[2];
    
  std::vector<Isometry3dTime> feet_posT;
  feet_posT.push_back( Isometry3dTime(msg->utime , left_pos  )  );
  feet_posT.push_back( Isometry3dTime(msg->utime+1 , right_pos  )  );
  pc_vis_->pose_collection_to_lcm_from_list(6003, feet_posT); 
}


int
main(int argc, char ** argv){
  /*
  ConciseArgs opt(argc, (char**)argv);
  opt.parse();*/
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM("") );
  if(!lcm->good())
    return 1;  
  
  leg_odometry app(lcm);
  while(0 == lcm->handle());
  return 0;
}
