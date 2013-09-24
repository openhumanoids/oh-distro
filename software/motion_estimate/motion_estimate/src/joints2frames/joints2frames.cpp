#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>

#include "joints2frames.hpp"
#include <ConciseArgs>

using namespace std;
using namespace boost;
using namespace boost::assign;

#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////

joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &lcm_, bool show_labels_, bool show_triads_,
  bool standalone_head_, bool ground_height_, bool bdi_motion_estimate_):
          lcm_(lcm_), show_labels_(show_labels_), show_triads_(show_triads_),
          standalone_head_(standalone_head_), ground_height_(ground_height_),
          bdi_motion_estimate_(bdi_motion_estimate_){
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

  pc_vis_->obj_cfg_list.push_back( obj_cfg(6003,"BDI Feet",5,1) );
  lcm_->subscribe("ATLAS_FOOT_POS_EST",&joints2frames::foot_pos_est_handler,this);  
  
  last_ground_publish_utime_ =0;
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

void joints2frames::publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel){
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


void joints2frames::publishRigidTransform(Eigen::Isometry3d pose, int64_t utime, std::string channel){
  bot_core::rigid_transform_t tf;
  tf.utime = utime;
  tf.trans[0] = pose.translation().x();
  tf.trans[1] = pose.translation().y();
  tf.trans[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  tf.quat[0] = quat.w();
  tf.quat[1] = quat.x();
  tf.quat[2] = quat.y();
  tf.quat[3] = quat.z();
  lcm_->publish(channel, &tf);    
}


void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  
  // 0. Extract World Pose of body:
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body.rotate(quat);    
  publishPose(world_to_body, msg->utime, "POSE_BODY" );
    
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
  

  // 2a. Determine the required BOT_FRAMES transforms:
  Eigen::Isometry3d body_to_head, body_to_hokuyo_link;
  bool body_to_head_found =false;
  bool body_to_hokuyo_link_found = false;

  for( map<string, KDL::Frame >::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string joint = (*ii).first;
    if (   (*ii).first.compare( "head" ) == 0 ){
      body_to_head = KDLToEigen( (*ii).second );
      body_to_head_found=true;
    }else if(  (*ii).first.compare( "hokuyo_link" ) == 0 ){
      body_to_hokuyo_link = KDLToEigen( (*ii).second );
      body_to_hokuyo_link_found=true;
    }else if(  (*ii).first.compare( "right_palm_left_camera_optical_frame" ) == 0 ){
      publishRigidTransform( KDLToEigen( (*ii).second ) , msg->utime, "BODY_TO_CAMERARHAND_LEFT" );
    }else if(  (*ii).first.compare( "left_palm_left_camera_optical_frame" ) == 0 ){
      publishRigidTransform( KDLToEigen( (*ii).second ) , msg->utime, "BODY_TO_CAMERALHAND_LEFT" );
    }
  }


  
  // 2b. Republish the required BOT_FRAMES transforms:
  if (body_to_head_found){
    /*
     * DONT PUBLISH THIS FOR SIMULATOR - CURRENTLY PUBLISHED BY LEGODO PROCESS
     */
    
    if (bdi_motion_estimate_){
      publishRigidTransform(body_to_head.inverse(), msg->utime, "HEAD_TO_BODY");
      
      Eigen::Isometry3d world_to_head = world_to_body * body_to_head ;
      publishPose(world_to_head, msg->utime, "POSE_HEAD" );
    }
    
    if (body_to_hokuyo_link_found){
      Eigen::Isometry3d head_to_hokuyo_link = body_to_head.inverse() * body_to_hokuyo_link ;
      publishRigidTransform(head_to_hokuyo_link, msg->utime, "HEAD_TO_HOKUYO_LINK");
    }
  }
  
  if (standalone_head_){
    // If publishing from the head alone, then the head is also the body link:
      publishRigidTransform(body_to_hokuyo_link, msg->utime, "HEAD_TO_HOKUYO_LINK" ); 
  }
  
  if (ground_height_){ 
    // Publish a pose at the lower of the feet - assumed to be on the ground
    // TODO: This doesnt need to be published at 1000Hz
    Eigen::Isometry3d body_to_l_foot = KDLToEigen(cartpos_out.find("l_foot")->second);
    Eigen::Isometry3d body_to_r_foot = KDLToEigen(cartpos_out.find("r_foot")->second);

    Eigen::Isometry3d foot_to_sole;
    foot_to_sole.setIdentity();
    foot_to_sole.translation()  << 0.0,0.,-0.0811; //distance between foot link and sole of foot
    
    Eigen::Isometry3d world_to_l_sole = world_to_body * body_to_l_foot * foot_to_sole;
    Eigen::Isometry3d world_to_r_sole = world_to_body * body_to_r_foot * foot_to_sole;

    // Publish lower of the soles at the ground occasionally
    if (msg->utime - last_ground_publish_utime_  > 5E5){ // every 0.5sec
      last_ground_publish_utime_ =msg->utime;
      if ( world_to_l_sole.translation().z() < world_to_r_sole.translation().z() ){
        publishPose(world_to_l_sole, msg->utime,"POSE_GROUND");
      }else{
        publishPose(world_to_r_sole, msg->utime,"POSE_GROUND");
      }
    }
  }
  
  Eigen::Isometry3d body_to_utorso = KDLToEigen(cartpos_out.find("utorso")->second);
  publishRigidTransform(body_to_utorso, msg->utime, "BODY_TO_UTORSO");
  

  // 4. Loop through joints and extract world positions:
  if (show_triads_){
    int counter =msg->utime;  
    std::vector<Isometry3dTime> body_to_jointTs, world_to_jointsT;
    std::vector< int64_t > body_to_joint_utimes;
    std::vector< std::string > joint_names;
    for( map<string, KDL::Frame >::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
      std::string joint = (*ii).first;
      //cout << joint  << ": \n";
      joint_names.push_back( joint  );
      body_to_joint_utimes.push_back( counter);
      
      Eigen::Isometry3d body_to_joint = KDLToEigen( (*ii).second );
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
}


// Visualize the foot positions from BDI:
// This can be turnned off if necessary - its not important
void joints2frames::foot_pos_est_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_foot_pos_est_t* msg){
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
  string role = "robot";
  bool labels = false;
  bool triads = false;
  bool standalone_head = false;
  bool ground_height = false;
  bool bdi_motion_estimate = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(role, "r", "role","Role - robot or base");
  opt.add(triads, "t", "triads","Frame Triads - show no not");
  opt.add(labels, "l", "labels","Frame Labels - show no not");
  opt.add(ground_height, "g", "ground", "Publish the grounded foot pose");
  opt.add(standalone_head, "s", "standalone_head","Standalone Sensor Head");
  opt.add(bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make frames [Temporary!]");
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
  
  joints2frames app(lcm,labels,triads, standalone_head, ground_height, bdi_motion_estimate);
  while(0 == lcm->handle());
  return 0;
}
