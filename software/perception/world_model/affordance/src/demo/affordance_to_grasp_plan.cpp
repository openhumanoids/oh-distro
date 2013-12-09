// Working Grasps:
// Box: both irobot and sandia - no edge cases. click on top and bottom of face to orientate the fingers
// Cylinder: both - assumes that z-axis of cylinder is upwards
// Steering Cyl: both. works for both directions. irobot has addition feature of making a flat face if you click at the center.

#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.hpp>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <pointcloud_tools/pointcloud_math.hpp>

#include <affordance/AffordanceUtils.hpp>

#include <ConciseArgs>
using namespace Eigen;




using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;  
    bool cartpos_ready_;
    
    void affHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::affordance_plus_collection_t* msg);      
    void initGraspHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::grasp_opt_control_t* msg);
    void planHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg);    
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);
    void poseGroundHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);

    void planGraspBox(Eigen::Isometry3d init_grasp_pose);
    void planGraspBoxIrobot(Eigen::Isometry3d init_grasp_pose);
    void planGraspBoxSandia(Eigen::Isometry3d init_grasp_pose);
    
    void planGraspSteeringCylinder(Eigen::Isometry3d init_grasp_pose);
    void planGraspCylinder(Eigen::Isometry3d init_grasp_pose);
    void planGraspFirehose(Eigen::Isometry3d init_grasp_pose);
    
    void sendCandidateGrasp(Eigen::Isometry3d aff_to_palmgeometry, double rel_angle);
    void sendStandingPositionValve(drc::affordance_t aff);
    void sendStandingPositionWye(drc::affordance_t aff);
    void sendStandingPositionFirehose(drc::affordance_t aff);
    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;
    
    // Affordances stored using the object_name that Sisir seems to be sending with INIT_GRASP_OPT_* messages
    // [otdf_type]_[uid]
    map<string, drc::affordance_t > affs_;
    
    drc::affordance_t aff_;
    Eigen::Isometry3d world_to_aff_ ;
    Eigen::Isometry3d world_to_body_;
    double ground_height_;
    
    AffordanceUtils affutils_;
    
    vector<string> sandia_l_joint_name_;
    vector<string> sandia_r_joint_name_;
    vector<double> sandia_l_joint_position_;    
    vector<double> sandia_r_joint_position_;
    
    vector<string> irobot_l_joint_name_;
    vector<string> irobot_r_joint_name_;
    vector<double> irobot_l_joint_position_;    
    vector<double> irobot_r_joint_position_;    
    
    vector<string> robotiq_l_joint_name_;
    vector<string> robotiq_r_joint_name_;
    vector<double> robotiq_l_joint_position_;    
    vector<double> robotiq_r_joint_position_;    
    
    std::vector <Isometry3dTime> eeloci_poses_;
    bool eeloci_plan_outstanding_;    
    
    drc::grasp_opt_control_t grasp_opt_msg_;

    bool verbose_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){
  verbose_ = true;
  cartpos_ready_ = false;
      
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = new KDL::TreeFkSolverPosFull_recursive(tree);
      
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robot_state_handler,this);  
  lcm_->subscribe("CANDIDATE_MANIP_PLAN",&Pass::planHandler,this);  
  
  lcm_->subscribe( "AFFORDANCE_PLUS_COLLECTION" ,&Pass::affHandler,this);
  lcm_->subscribe( "INIT_GRASP_OPT_1" ,&Pass::initGraspHandler,this);
  lcm_->subscribe( "INIT_GRASP_OPT_2" ,&Pass::initGraspHandler,this);

  lcm_->subscribe( "POSE_GROUND" ,&Pass::poseGroundHandler,this);
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60011,"Grasp Seed",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60001,"Grasp Frame",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60002,"Palm Frame",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60003,"Corner Frame",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60012,"Grasp Pose Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60013,"Grasp Feasibility" ,1,1, 60012,0, { 0.0, 1.0, 0.0} ));
  //
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60014,"Standing Position",5,1) ); // 4 is pose3d

  sandia_l_joint_name_ = {"left_f0_j0","left_f0_j1","left_f0_j2",   "left_f1_j0","left_f1_j1","left_f1_j2",
    "left_f2_j0","left_f2_j1","left_f2_j2",   "left_f3_j0","left_f3_j1","left_f3_j2" };
  sandia_l_joint_position_ = {0,0,0,  0,0,0,                        0,0,0,  0,0,0};
  sandia_r_joint_name_ = {"right_f0_j0","right_f0_j1","right_f0_j2",  "right_f1_j0","right_f1_j1","right_f1_j2",
    "right_f2_j0","right_f2_j1","right_f2_j2",  "right_f3_j0","right_f3_j1","right_f3_j2" };
  // fore, middle, little, thumb | lower mid upper
  sandia_r_joint_position_ = {0,0,0,  0,0,0, 0,0,0,  0,0,0};


  irobot_l_joint_name_ = {"left_finger[0]/joint_base_rotation", "left_finger[0]/joint_base", "left_finger[0]/joint_flex",
      "left_finger[1]/joint_base_rotation", "left_finger[1]/joint_base", "left_finger[1]/joint_flex",
      "left_finger[2]/joint_base", "left_finger[2]/joint_flex" };
  irobot_l_joint_position_ = { 0, 0, 0,         0, 0, 0,         0, 0};
  
  
  irobot_r_joint_name_ = {"right_finger[0]/joint_base_rotation", "right_finger[0]/joint_base", "right_finger[0]/joint_flex",
      "right_finger[1]/joint_base_rotation", "right_finger[1]/joint_base", "right_finger[1]/joint_flex",
      "right_finger[2]/joint_base", "right_finger[2]/joint_flex" };
  irobot_r_joint_position_ = { 0, 0, 0,         0, 0, 0,         0, 0};
  
  
  
  robotiq_l_joint_name_ = { "left_finger_1_joint_1", "left_finger_1_joint_2", "left_finger_1_joint_3",
        "left_finger_2_joint_1", "left_finger_2_joint_2", "left_finger_2_joint_3",
        "left_finger_middle_joint_1", "left_finger_middle_joint_2", "left_finger_middle_joint_3",
        "left_palm_finger_1_joint", "left_palm_finger_2_joint"};
  robotiq_l_joint_position_ = { 0, 0, 0,         0,0,0,         0,0,0   ,0,0};
  robotiq_r_joint_name_ = { "right_finger_1_joint_1", "right_finger_1_joint_2", "right_finger_1_joint_3",
        "right_finger_2_joint_1", "right_finger_2_joint_2", "right_finger_2_joint_3",
        "right_finger_middle_joint_1", "right_finger_middle_joint_2", "right_finger_middle_joint_3",
        "right_palm_finger_1_joint", "right_palm_finger_2_joint"};  
  robotiq_r_joint_position_ = { 0, 0, 0,         0,0,0,         0,0,0   ,0,0};
  
  eeloci_plan_outstanding_ = false;
}

void Pass::poseGroundHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  ground_height_ = msg->pos[2];
}

void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  
  // 0. Extract World Pose of body:
  world_to_body_.setIdentity();
  world_to_body_.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body_.rotate(quat);    
    
  // 1. Solve for Forward Kinematics:
  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  cartpos_.clear();
  for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));
  
  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos_,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  cartpos_ready_=true;
}


// Draw the standing position for the valve task:
void Pass::sendStandingPositionValve(drc::affordance_t aff){ 
  
  
  // cylinder aff main axis is z-axis, determine yaw in world frame of that axis:
  Eigen::Quaterniond q1=  euler_to_quat( aff.origin_rpy[0] ,  aff.origin_rpy[1] , aff.origin_rpy[2]  ); 
  Eigen::Quaterniond q2=  euler_to_quat( 0 ,  -90*M_PI/180 , 0  ); 
  q1= q1*q2;  
  double look_rpy[3];
  quat_to_euler ( q1, look_rpy[0], look_rpy[1], look_rpy[2] );
  ///////////////////////////////////////

  int counter = 0;
  std::vector<Isometry3dTime>  feet_positionsT;

  for (int front_side=0; front_side < 2; front_side++){ // was zero
    Eigen::Isometry3d valve_pose(Eigen::Isometry3d::Identity());
    valve_pose.translation()  << aff.origin_xyz[0], aff.origin_xyz[1], ground_height_;
    if (front_side){
      valve_pose.rotate( Eigen::Quaterniond(  euler_to_quat( 0 ,  0 ,  look_rpy[2] )  ) );
    }else{
      valve_pose.rotate( Eigen::Quaterniond(  euler_to_quat( 0 ,  0 ,  look_rpy[2] + M_PI )  ) );
    }
    feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose) );

    for (int left_reach =0 ; left_reach<2 ; left_reach++){
      int left_reach_sign =1;
      if (left_reach){
        left_reach_sign = -1;
      }
      
      Eigen::Isometry3d valve2com(Eigen::Isometry3d::Identity());
      if (1==0){ // old place for sandia hands - still works
        valve2com.translation()  << -0.45, left_reach_sign*-0.28, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,left_reach_sign*20*M_PI/180))  );   
      }else{ // iRobot  hand or pointer
        valve2com.translation()  << -0.65, left_reach_sign*-0.45, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,0*M_PI/180))  );   
      }
        
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com) );
      
      // Draw foot positions:
      /* Eigen::Isometry3d com2left(Eigen::Isometry3d::Identity());
      com2left.translation()  << 0.0, 0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2left) );
      Eigen::Isometry3d com2right(Eigen::Isometry3d::Identity());
      com2right.translation()  << 0.0, -0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2right) );
      */

    }
  }

  pc_vis_->pose_collection_to_lcm_from_list(60014, feet_positionsT); 
}



void Pass::sendStandingPositionWye(drc::affordance_t aff){ 
  
  // cylinder aff main axis is z-axis, determine yaw in world frame of that axis:
  Eigen::Quaterniond q1=  euler_to_quat( aff.origin_rpy[0] ,  aff.origin_rpy[1] , aff.origin_rpy[2]  ); 
  double look_rpy[3];
  quat_to_euler ( q1, look_rpy[0], look_rpy[1], look_rpy[2] );
  ///////////////////////////////////////

  int counter = 0;
  std::vector<Isometry3dTime>  feet_positionsT;

    Eigen::Isometry3d valve_pose(Eigen::Isometry3d::Identity());
    valve_pose.translation()  << aff.origin_xyz[0], aff.origin_xyz[1], ground_height_;
    valve_pose.rotate( Eigen::Quaterniond(  euler_to_quat( 0 ,  0 ,  look_rpy[2] )  ) );
    feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose) );

    for (int left_reach = 0; left_reach<2 ; left_reach++){
      Eigen::Isometry3d valve2com(Eigen::Isometry3d::Identity());
      if (left_reach){
        valve2com.translation()  << -0.40, 0.59, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,-35*M_PI/180))  );   
      }else{
        valve2com.translation()  << -0.40, -0.59, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,35*M_PI/180 ))  );   
      }
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com) );

      /*
      Eigen::Isometry3d com2left(Eigen::Isometry3d::Identity());
      com2left.translation()  << 0.0, 0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2left) );
      Eigen::Isometry3d com2right(Eigen::Isometry3d::Identity());
      com2right.translation()  << 0.0, -0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2right) );
      */

    }

  pc_vis_->pose_collection_to_lcm_from_list(60014, feet_positionsT); 
}


void Pass::sendStandingPositionFirehose(drc::affordance_t aff){ 
  
  // cylinder aff main axis is z-axis, determine yaw in world frame of that axis:
  Eigen::Quaterniond q1=  euler_to_quat( aff.origin_rpy[0] ,  aff.origin_rpy[1] , aff.origin_rpy[2]  ); 
  double look_rpy[3];
  quat_to_euler ( q1, look_rpy[0], look_rpy[1], look_rpy[2] );
  ///////////////////////////////////////

  int counter = 0;
  std::vector<Isometry3dTime>  feet_positionsT;

    Eigen::Isometry3d valve_pose(Eigen::Isometry3d::Identity());
    valve_pose.translation()  << aff.origin_xyz[0], aff.origin_xyz[1], ground_height_;
    valve_pose.rotate( Eigen::Quaterniond(  euler_to_quat( 0 ,  0 ,  look_rpy[2] )  ) );
    feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose) );

    for (int left_reach = 0; left_reach<2 ; left_reach++){
      Eigen::Isometry3d valve2com(Eigen::Isometry3d::Identity());
      if (left_reach){
        valve2com.translation()  << -0.5406, 0, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,0))  );   
      }else{
        valve2com.translation()  << -0.5406, 0, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,0 ))  );   
        // was -0.53 -0.53  and 18 degrees for first trial runthrough
      }
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com) );

      /*
      Eigen::Isometry3d com2left(Eigen::Isometry3d::Identity());
      com2left.translation()  << 0.0, 0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2left) );
      Eigen::Isometry3d com2right(Eigen::Isometry3d::Identity());
      com2right.translation()  << 0.0, -0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2right) );
      */

    }

  pc_vis_->pose_collection_to_lcm_from_list(60014, feet_positionsT); 
}


void Pass::affHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  //std::cout << "got "<< msg->naffs << " affs\n";
  
  
  affs_.clear();
  for (int i=0 ; i < msg->naffs ; i++){
    drc::affordance_t aff = msg->affs_plus[i].aff;
    std::stringstream ss;
    ss << aff.otdf_type << '_' << aff.uid;    
    // std::cout << ss.str() << "\n";

    affs_[ ss.str() ]=  aff;
    
    if (aff.otdf_type == "steering_cyl"){
      sendStandingPositionValve( aff );
    }else if (aff.otdf_type == "wye"){
      sendStandingPositionWye( aff );
    }else if (aff.otdf_type == "firehose"){
      sendStandingPositionFirehose( aff );
    }
  }    
}

void Pass::planHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg){
  if (!eeloci_plan_outstanding_){
    return;
  }
  std::cout << "got a plan\n";
  
  if ( (int) eeloci_poses_.size()  != msg->num_states ){
    std::cout << eeloci_poses_.size() <<" n. loci is not number of plan states ["<<msg->num_states<<"]. will need to solve FK\n"; 
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud->width   = msg->num_states;
  cloud->height   = 1;
  cloud->points.resize (msg->num_states);
  for (int i = 0; i < msg->num_states; i++) {
    cloud->points[i].x = eeloci_poses_[i].pose.translation().x();
    cloud->points[i].y = eeloci_poses_[i].pose.translation().y();
    cloud->points[i].z = eeloci_poses_[i].pose.translation().z();
    if (msg->plan_info[i] < 10){
      cloud->points[i].r = 0; cloud->points[i].g = 255; cloud->points[i].b = 0;
    }else{
      cloud->points[i].r = 255; cloud->points[i].g = 0; cloud->points[i].b = 0;
    }
  }
  
  // Plot scan in local frame:
  Isometry3dTime null_poseT = Isometry3dTime(msg->utime, Eigen::Isometry3d::Identity());
  pc_vis_->pose_to_lcm_from_list(60012, null_poseT);  
  pc_vis_->ptcld_to_lcm_from_list(60013, *cloud, null_poseT.utime, null_poseT.utime);  
  
  eeloci_plan_outstanding_ = false;
  eeloci_poses_.clear();  
}


/// Above sets the process state.
/// Below is all thats reactive
void Pass::initGraspHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::grasp_opt_control_t* msg){
  /*
   * if ( !cartpos_ready_ ){
    std::cout << "State Not received yet\n";
    return; 
  }*/
  
  std::cout << "got grasp\n";
  
  std::vector<Isometry3dTime> init_grasp_poseT;
  Eigen::Isometry3d init_grasp_pose = Eigen::Isometry3d::Identity();
  init_grasp_pose.translation() << msg->ray_hit[0], msg->ray_hit[1], msg->ray_hit[2];
  init_grasp_poseT.push_back( Isometry3dTime(msg->utime, init_grasp_pose) );
  if (verbose_)  pc_vis_->pose_collection_to_lcm_from_list(60011, init_grasp_poseT); 
  
  grasp_opt_msg_ = *msg; // need this to creat output
  
  
  map< string , drc::affordance_t >::iterator it = affs_.find( grasp_opt_msg_.object_name );
  if (it == affs_.end() ){
    std::cout << "couldn't find affordance of name ["<< grasp_opt_msg_.object_name << "]\n";
    return;
  }else{
    aff_ = it->second;
  }

  std::cout << "Find grasp to " <<  grasp_opt_msg_.object_name << "\n";
  

  std::cout << aff_.otdf_type << "\n";
  if ( aff_.otdf_type == "box" ){
    planGraspBox(init_grasp_pose);  
  }else if( aff_.otdf_type == "steering_cyl"  ){
    planGraspSteeringCylinder(init_grasp_pose);
  }else if( aff_.otdf_type == "cylinder"  ){
    planGraspCylinder(init_grasp_pose);
  }else if( aff_.otdf_type == "firehose"  ){
    planGraspFirehose(init_grasp_pose);
  }else{
    std::cout << "no grasping method for a ["<<  aff_.otdf_type << "\n";
  }

  
  // This is required to spoof the aff server
  drc::grasp_opt_status_t msg_g;
  msg_g.utime = 0;
  msg_g.matlab_pool_ready=1;
  msg_g.num_matlab_workers=2;
  msg_g.worker_available=0  ;
  msg_g.worker_id = 1;
  lcm_->publish("GRASP_OPT_STATUS",&msg_g );
  msg_g.worker_id = 2;
  lcm_->publish("GRASP_OPT_STATUS",&msg_g );  
}



void Pass::planGraspFirehose(Eigen::Isometry3d init_grasp_pose){  
  
  // 1. Determine the Parameters of the Cylinder we need:
  // Radius and Relative Angle  
  world_to_aff_ = affutils_.getPose(aff_.origin_xyz, aff_.origin_rpy);
  Eigen::Affine3d Aff = Eigen::Affine3d(world_to_aff_.inverse() );
  Eigen::Vector3d pt = Eigen::Vector3d(init_grasp_pose.translation().x(), 
                                       init_grasp_pose.translation().y(),
                                       init_grasp_pose.translation().z());
  pt = Aff * pt;
  double rel_angle = atan2(pt(1), pt(0)  );
  
  std::map<string,double> am;
  for (int j=0; j< aff_.nparams; j++){
    am[ aff_.param_names[j] ] = aff_.params[j];
  }
  double radius =0.0266;
  
  std::cout << rel_angle*180/M_PI << " Relative Angle around Cylinder\n";
  std::cout <<  radius << " Radius\n";
  
  
  // 2. Create a reasonable afforance to hand pose 
  Eigen::Isometry3d aff_to_palmgeometry = Eigen::Isometry3d::Identity();
  // translation on cylinder:
  // outwards, backward, updown 

  
  int flip_sign = 1;
  if (pt(2) < 0 ){
    std::cout << "Flip hand\n";
    aff_to_palmgeometry.rotate( euler_to_quat(180*M_PI/180, 0,0  ) );   
    flip_sign=-1;
  }
  
  
  // Sandia
  // was: 0.05 + radius ,0,-0.12;
  // 0.03 was too little
  if (grasp_opt_msg_.grasp_type ==0){ // sandia left
    aff_to_palmgeometry.translation()  << 0.05 + radius , flip_sign*(0.06 + 0.4*radius),0;
    aff_to_palmgeometry.rotate( euler_to_quat(75*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
  }else if (grasp_opt_msg_.grasp_type ==1){ // sandia right
    aff_to_palmgeometry.translation()  << (0.05 + radius) ,-flip_sign* (0.06 + 0.4*radius),0.0;
    aff_to_palmgeometry.rotate( euler_to_quat( -75*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
  }else if (grasp_opt_msg_.grasp_type ==3){ // irobot left
    aff_to_palmgeometry.translation()  << 0.095 + radius ,0,0;
    aff_to_palmgeometry.rotate( euler_to_quat(90*M_PI/180, 0,0  ) );   
  }else if (grasp_opt_msg_.grasp_type ==4){ // irobot right
    aff_to_palmgeometry.translation()  << 0.095 + radius ,0,0;
    aff_to_palmgeometry.rotate( euler_to_quat(90*M_PI/180, 0,0  ) );   
  }
  
  
  // Offset up and down the cylinder - not used for firehose
//  aff_to_palmgeometry.translation() += Eigen::Vector3d(0,0, pt(2) );
  
  sendCandidateGrasp(aff_to_palmgeometry, rel_angle);
}



void Pass::planGraspCylinder(Eigen::Isometry3d init_grasp_pose){  
  
  // 1. Determine the Parameters of the Cylinder we need:
  // Radius and Relative Angle  
  world_to_aff_ = affutils_.getPose(aff_.origin_xyz, aff_.origin_rpy);
  Eigen::Affine3d Aff = Eigen::Affine3d(world_to_aff_.inverse() );
  Eigen::Vector3d pt = Eigen::Vector3d(init_grasp_pose.translation().x(), 
                                       init_grasp_pose.translation().y(),
                                       init_grasp_pose.translation().z());
  pt = Aff * pt;
  double rel_angle = atan2(pt(1), pt(0)  );
  
  std::map<string,double> am;
  for (int j=0; j< aff_.nparams; j++){
    am[ aff_.param_names[j] ] = aff_.params[j];
  }
  double radius = am.find("radius")->second;
  
  std::cout << rel_angle*180/M_PI << " Relative Angle around Cylinder\n";
  std::cout <<  radius << " Radius\n";
  
  
  // 2. Create a reasonable afforance to hand pose 
  Eigen::Isometry3d aff_to_palmgeometry = Eigen::Isometry3d::Identity();
  // translation on cylinder:
  // outwards, backward, updown 

  // Sandia
  // was: 0.05 + radius ,0,-0.12;
  // 0.03 was too little
  if (grasp_opt_msg_.grasp_type ==0){ // sandia left
    aff_to_palmgeometry.translation()  << 0.05 + radius ,0.06 + 0.4*radius,0;
    aff_to_palmgeometry.rotate( euler_to_quat(75*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
    
//    aff_to_palmgeometry.rotate( euler_to_quat(-15*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
  }else if (grasp_opt_msg_.grasp_type ==1){ // sandia right
    aff_to_palmgeometry.translation()  << 0.05 + radius ,-(0.06 + 0.4*radius),0.0;
    aff_to_palmgeometry.rotate( euler_to_quat( -75*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
  }else if (grasp_opt_msg_.grasp_type ==3){ // irobot left
    aff_to_palmgeometry.translation()  << 0.095 + radius ,0,0;
    aff_to_palmgeometry.rotate( euler_to_quat(90*M_PI/180, 0,0  ) );   
  }else if (grasp_opt_msg_.grasp_type ==4){ // irobot right
    aff_to_palmgeometry.translation()  << 0.095 + radius ,0,0;
    aff_to_palmgeometry.rotate( euler_to_quat(90*M_PI/180, 0,0  ) );   
    std::cout << "irobot right\n";
  }
  
  // Offset up and down the cylinder:
  aff_to_palmgeometry.translation() += Eigen::Vector3d(0,0, pt(2) );
  
  sendCandidateGrasp(aff_to_palmgeometry, rel_angle);
}



void Pass::planGraspBox(Eigen::Isometry3d init_grasp_pose){  
  if ( (grasp_opt_msg_.grasp_type ==0) || (grasp_opt_msg_.grasp_type ==1) ) { // Sandia left or right_f0_j0
    std::cout << "Grasp Box with Sandia left or right\n";
    planGraspBoxSandia(init_grasp_pose);
  }else if ( (grasp_opt_msg_.grasp_type ==3) || (grasp_opt_msg_.grasp_type ==4) ) { // iRobot left or right
    std::cout << "Grasp Box with iRobot left or right\n";
    planGraspBoxIrobot(init_grasp_pose);
  }else if ( (grasp_opt_msg_.grasp_type ==6) || (grasp_opt_msg_.grasp_type ==7) ) { // Robotiq left or right
    std::cout << "Grasp Box with Robotiq left or right\n";
    planGraspBoxIrobot(init_grasp_pose);
  }
  
}


void Pass::planGraspBoxIrobot(Eigen::Isometry3d init_grasp_pose){  

  std::map<string,double> am;
  for (int j=0; j< aff_.nparams; j++){
    am[ aff_.param_names[j] ] = aff_.params[j];
  }
  Eigen::Vector3d aff_len( am.find("lX")->second, am.find("lY")->second, am.find("lZ")->second );
  
  
  // 1. Determine the Parameters:
  world_to_aff_ = affutils_.getPose(aff_.origin_xyz, aff_.origin_rpy);
  Eigen::Affine3d Aff = Eigen::Affine3d(world_to_aff_.inverse() );
  Eigen::Vector3d grasp_point = Eigen::Vector3d(init_grasp_pose.translation().x(), 
                                       init_grasp_pose.translation().y(),
                                       init_grasp_pose.translation().z());
  grasp_point = Aff * grasp_point; // relative grasp point
  Eigen::Vector3d hit_point = grasp_point;

  std::cout << "grasp_point: " << grasp_point.transpose() << "\n";
  std::cout << "aff_len: " << aff_len.transpose() << "\n";
  
  double distance_to_x_face = fabs( fabs (grasp_point(0)) - aff_len(0)/2 );
  double distance_to_y_face = fabs( fabs (grasp_point(1)) - aff_len(1)/2 );  
  double distance_to_z_face = fabs( fabs (grasp_point(2)) - aff_len(2)/2 );
  
  int face_dim = 0; // the dimension that the hand is facing
  double distance_to_face_smallest = distance_to_x_face;
  if (distance_to_y_face  < distance_to_face_smallest){
    face_dim = 1;
    distance_to_face_smallest = distance_to_y_face;
  }

  if (distance_to_z_face  < distance_to_face_smallest){
    face_dim = 2;
    distance_to_face_smallest = distance_to_z_face;
  }
  
  std::cout << "0: " << " | " << distance_to_x_face << "\n";
  std::cout << "1: " << " | " << distance_to_y_face << "\n";
  std::cout << "2: " << " | " << distance_to_z_face << "\n";
  Eigen::Vector3d distances_to_face(distance_to_x_face,distance_to_y_face,distance_to_z_face);
  
  int long_dim=-1;
  int short_dim=-1;
  if (face_dim==0){
    if ( aff_len(1)  > aff_len(2) ){
      std::cout << "a\n";
      long_dim = 1;
      short_dim = 2;
    }else{
      std::cout << "b\n";
      long_dim = 2;
      short_dim = 1;
    }
  }else if (face_dim==1){
    if ( aff_len(0)  > aff_len(2) ){
      long_dim = 0;
      short_dim = 2;
    }else{
      long_dim = 2;
      short_dim = 0;
    }
  }else if(face_dim==2){
    if ( aff_len(0)  > aff_len(1) ){
      long_dim = 0;
      short_dim = 1;
    }else{
      long_dim = 1;
      short_dim = 0;
    }
  }
  
  
  
  
  Eigen::Vector3d fingers_rotation (0,0,0); 
  if ( grasp_point ( short_dim ) < 0 ){
    std::cout << "fingers down\n";
    fingers_rotation << M_PI,0,0; 
  }else{
    std::cout << "fingers up\n"; 
    fingers_rotation << 0,0,0; 
  }
  
  // added nov 3, not sure why needed
  // but hands didn't work properly with pat's box affordances
  if(long_dim ==2){ 
    std::cout << "extra rotation bit\n";
    fingers_rotation(0) += M_PI/2; 
    if (face_dim ==0){
      if ( grasp_point( face_dim ) < 0){
        fingers_rotation(0) -= M_PI; 
      }else{
        //fingers_rotation(0) += 0; 
      }
    }
  }
  
  
  grasp_point(short_dim) =0;
  grasp_point(face_dim) = copysign(1, grasp_point(face_dim) ) * aff_len(face_dim)/2; // snap onto face exactly
      
  std::cout << "face dim: " << face_dim << " | " << distance_to_face_smallest << "\n";
  std::cout << "long_dim: " << long_dim << "\n";

  
  Eigen::Vector3d face_rotation (0,0,0); 
  Eigen::Vector3d direction_rotation (0,0,0); 
  if (face_dim == 1 ) {
     face_rotation << 0,0, M_PI/2 ;
  }else if (face_dim == 2 ) {
//     face_rotation << M_PI, -M_PI/2 , 0 ;
     face_rotation << 0, -M_PI/2 , -M_PI/2 ;
  }
  
  if ( grasp_point( face_dim ) > 0){
    std::cout << "positive, need flip\n";
    direction_rotation << 0 , 0, M_PI;
  }
  
  
  Eigen::Isometry3d aff_to_actualpalm = Eigen::Isometry3d::Identity();
  aff_to_actualpalm.rotate( euler_to_quat( face_rotation(0) , face_rotation(1),  face_rotation(2) ) );   
  
  {
  std::cout << "face_rotation: " << face_rotation.transpose() << "\n";
  std::stringstream ss;
  print_Isometry3d(aff_to_actualpalm, ss);
  std::cout << "0 aff_to_actualpalm: " << ss.str() << "\n";
  }  
  
  aff_to_actualpalm.rotate( euler_to_quat( direction_rotation(0) , direction_rotation(1),  direction_rotation(2) ) );   
  
  {
  std::cout << "direction_rotation: " << direction_rotation.transpose() << "\n";
  std::stringstream ss;
  print_Isometry3d(aff_to_actualpalm, ss);
  std::cout << "A aff_to_actualpalm: " << ss.str() << "\n";
  }  
  
  aff_to_actualpalm.rotate( euler_to_quat( fingers_rotation(0) , fingers_rotation(1),  fingers_rotation(2) ) );   

  Eigen::Quaterniond r= euler_to_quat( fingers_rotation(0) , fingers_rotation(1),  fingers_rotation(2) );
  std::cout << "fingers quat: " << r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() << "\n";
  
  std::cout << "fingers_rotation: " << fingers_rotation.transpose() << "\n";
  
  //if(face_dim==2){
  //  std::cout << "awkward error case on dim 2 correction\n";
  //  aff_to_actualpalm.rotate( euler_to_quat( -M_PI/2 , 0, 0 ) );   
  //}
  
  aff_to_actualpalm.translation()  << grasp_point(0) , grasp_point(1) , grasp_point(2) ;
  
  
  {
  std::stringstream ss;
  print_Isometry3d(aff_to_actualpalm, ss);
  std::cout << "B aff_to_actualpalm: " << ss.str() << "\n";
  }
  
  
  if ( fabs(aff_len(face_dim)-aff_len(short_dim)) < 0.02){
    if ( (fabs(distances_to_face(face_dim ))< 0.02)&& 
         (fabs(distances_to_face(short_dim))< 0.02)   ){
      std::cout << "diamond grasp of a square block\n";
      Eigen::Isometry3d tf_to_corner = Eigen::Isometry3d::Identity();
      if ((face_dim==1)&& (long_dim==2)) {
        if (grasp_point( face_dim ) >0 ){
          tf_to_corner.translation().z() = -aff_len(face_dim)/2;
          tf_to_corner.rotate( euler_to_quat( 0 , -M_PI/4,0  ) );   
        }else{
          tf_to_corner.translation().z() = aff_len(face_dim)/2;
          tf_to_corner.rotate( euler_to_quat( M_PI , M_PI/4,  0 ) );   
        }
      }else{
        tf_to_corner.translation().z() = aff_len(face_dim)/2;
        tf_to_corner.rotate( euler_to_quat( M_PI , M_PI/4,  0 ) );   
      }
      aff_to_actualpalm =aff_to_actualpalm*tf_to_corner;    
    }
  }
  
  std::stringstream ss;
  print_Isometry3d(world_to_aff_, ss);
  std::cout << "World to Aff: " << ss.str() << "\n";
  
  {
  std::stringstream ss;
  print_Isometry3d(aff_to_actualpalm, ss);
  std::cout << "C aff_to_actualpalm: " << ss.str() << "\n";
  }

  
  eeloci_poses_.clear();
  eeloci_poses_.push_back( Isometry3dTime(0, world_to_aff_* aff_to_actualpalm));

  // Add on the transform from the "actual palm" to the palm link
  Eigen::Isometry3d actualplam_to_palmgeometry = Eigen::Isometry3d::Identity();
  
  actualplam_to_palmgeometry.rotate( euler_to_quat( 0*M_PI/180, 0*M_PI/180, 180*M_PI/180 ) );
  actualplam_to_palmgeometry.translation()  <<  -0.115 , 0 , 0.0  ;   // + x + y + z
  
  Eigen::Isometry3d aff_to_palmgeometry = aff_to_actualpalm*actualplam_to_palmgeometry;
  
  if (verbose_) pc_vis_->pose_collection_to_lcm_from_list(60001, eeloci_poses_); 
  
  sendCandidateGrasp(aff_to_palmgeometry, 0);  
}



void Pass::planGraspBoxSandia(Eigen::Isometry3d init_grasp_pose){  

  std::map<string,double> am;
  for (int j=0; j< aff_.nparams; j++){
    am[ aff_.param_names[j] ] = aff_.params[j];
  }
  Eigen::Vector3d aff_len( am.find("lX")->second, am.find("lY")->second, am.find("lZ")->second );
  
  
  // 1. Determine the Parameters:
  world_to_aff_ = affutils_.getPose(aff_.origin_xyz, aff_.origin_rpy);
  Eigen::Affine3d Aff = Eigen::Affine3d(world_to_aff_.inverse() );
  Eigen::Vector3d grasp_point = Eigen::Vector3d(init_grasp_pose.translation().x(), 
                                       init_grasp_pose.translation().y(),
                                       init_grasp_pose.translation().z());
  grasp_point = Aff * grasp_point; // relative grasp point
  
  std::cout << "grasp_point: " << grasp_point.transpose() << "\n";
  std::cout << "aff_len: " << aff_len.transpose() << "\n";
  
  double distance_to_x_face = fabs( fabs (grasp_point(0)) - aff_len(0)/2 );
  double distance_to_y_face = fabs( fabs (grasp_point(1)) - aff_len(1)/2 );  
  double distance_to_z_face = fabs( fabs (grasp_point(2)) - aff_len(2)/2 );
  
  int face_dim = 0; // the dimension that the hand is facing
  double distance_to_face_smallest = distance_to_x_face;
  if (distance_to_y_face  < distance_to_face_smallest){
    face_dim = 1;
    distance_to_face_smallest = distance_to_y_face;
  }

  if (distance_to_z_face  < distance_to_face_smallest){
    face_dim = 2;
    distance_to_face_smallest = distance_to_z_face;
  }
  
  std::cout << "0: " << " | " << distance_to_x_face << "\n";
  std::cout << "1: " << " | " << distance_to_y_face << "\n";
  std::cout << "2: " << " | " << distance_to_z_face << "\n";
  
  int long_dim=-1;
  int short_dim=-1;
  if (face_dim==0){
    if ( aff_len(1)  > aff_len(2) ){
      std::cout << "a\n";
      long_dim = 1;
      short_dim = 2;
    }else{
      std::cout << "b\n";
      long_dim = 2;
      short_dim = 1;
    }
  }else if (face_dim==1){
    if ( aff_len(0)  > aff_len(2) ){
      long_dim = 0;
      short_dim = 2;
    }else{
      long_dim = 2;
      short_dim = 0;
    }
  }else if(face_dim==2){
    if ( aff_len(0)  > aff_len(1) ){
      long_dim = 0;
      short_dim = 1;
    }else{
      long_dim = 1;
      short_dim = 0;
    }
  }
  
  // fine to here

  std::cout << "face dim: " << face_dim << " | " << distance_to_face_smallest << "\n";
  std::cout << "long_dim: " << long_dim << "\n";  
  
  Eigen::Vector3d face_rotation (0,0,0); 
  Eigen::Vector3d direction_rotation (0,0,0); 
  if (face_dim == 1 ) {
     face_rotation << 0,0, M_PI/2 ;
  }else if (face_dim == 2 ) {
//     face_rotation << M_PI, -M_PI/2 , 0 ;
     face_rotation << 0, -M_PI/2 , -M_PI/2 ;
  }
  
  if (face_dim == 1 ) {
     face_rotation << 0,0, M_PI/2 ;
  }else if (face_dim == 2 ) {
//     face_rotation << M_PI, -M_PI/2 , 0 ;
     face_rotation << 0, -M_PI/2 ,0 ;
  }
  
  if ( grasp_point( face_dim ) > 0){
    std::cout << "positive, need flip\n";
    direction_rotation << M_PI , 0, M_PI;
  } else{
    direction_rotation << 0 , 0, 0;
  }
  
  
  Eigen::Vector3d fingers_rotation (0,0,0); 
  // added nov 3, not sure why needed
  // but hands didn't work properly with pat's box affordances
  if(long_dim ==2){ 
    std::cout << "extra rotation bit\n";
    fingers_rotation(0) += M_PI/2; 
    if (face_dim ==0){
      if ( grasp_point( face_dim ) < 0){
        fingers_rotation(0) -= M_PI; 
      }else{
        //fingers_rotation(0) += 0; 
      }
    }
  }  
  if(face_dim ==2){ 
    std::cout << "extra rotation bit\n";
    if (long_dim ==0){
      fingers_rotation(0) = M_PI/2; 
    }
  }  
  
  
  // palm always faces into box, fingers always point towards the short side
  
  
  
  Eigen::Isometry3d aff_to_actualpalm = Eigen::Isometry3d::Identity();
  aff_to_actualpalm.rotate( euler_to_quat( face_rotation(0) , face_rotation(1),  face_rotation(2) ) );   
  aff_to_actualpalm.rotate( euler_to_quat( direction_rotation(0) , direction_rotation(1),  direction_rotation(2) ) );   
  aff_to_actualpalm.rotate( euler_to_quat( fingers_rotation(0) , fingers_rotation(1),  fingers_rotation(2) ) );   
  
  
  /*
  Eigen::Vector3d fingers_rotation (0,0,0); 
  if ( grasp_point ( short_dim ) < 0 ){
    std::cout << "fingers down\n";
    fingers_rotation << 0,0,0; 
  }else{
    std::cout << "fingers up\n"; 
    fingers_rotation << M_PI,0,0; 
  }
  
      
  
  Eigen::Vector3d face_rotation (0,0,0); 

  
  Eigen::Isometry3d aff_to_actualpalm = Eigen::Isometry3d::Identity();
  aff_to_actualpalm.rotate( euler_to_quat( face_rotation(0) , face_rotation(1),  face_rotation(2) ) );   
  aff_to_actualpalm.rotate( euler_to_quat( direction_rotation(0) , direction_rotation(1),  direction_rotation(2) ) );   
  aff_to_actualpalm.rotate( euler_to_quat( fingers_rotation(0) , fingers_rotation(1),  fingers_rotation(2) ) );   
  
  */
  
  bool do_bottom_transform = 0;
  if (grasp_point(short_dim) > 0){
    std::cout << "top side of face\n";
    do_bottom_transform=0;
  }else{
    std::cout << "bottom side of face\n";
    do_bottom_transform=1;
  }
  
  //
  if ( (grasp_point(face_dim) < 0) && (face_dim==2) ) {
    do_bottom_transform = !do_bottom_transform;
  }
  if ( (grasp_point(face_dim) > 0) && (face_dim==1) ) {
    do_bottom_transform = !do_bottom_transform;
  }
  if ( (grasp_point(face_dim) > 0) && (face_dim==0) ) {
    do_bottom_transform = !do_bottom_transform;
  }
  
  
  grasp_point(short_dim) =0;
  grasp_point(face_dim) = copysign(1, grasp_point(face_dim) ) * aff_len(face_dim)/2; // snap onto face exactly
  
  
  aff_to_actualpalm.translation()  << grasp_point(0) , grasp_point(1) , grasp_point(2) ;   // + x + y + z
  // TODO support different params here
  
  eeloci_poses_.clear();
  eeloci_poses_.push_back( Isometry3dTime(0, world_to_aff_* aff_to_actualpalm));
  pc_vis_->pose_collection_to_lcm_from_list(60001, eeloci_poses_); 

  
  // at this stage aff_to_actualpalm is a frame:
  // - position in the middle of the board 
  // - x-axis pointing into the face (and long the finger)
  // - y-axis pointing towards knuckle on thump
  // - z-axis pointing along the 3 fingers axis
  
  // palm facing in positive direction of face_dim
  Eigen::Isometry3d aff_to_corner = Eigen::Isometry3d::Identity();
  // -0.055, 0 , - 0.06
  
  if (do_bottom_transform){
    aff_to_corner.translation() = Eigen::Vector3d( 0 , 0 ,  -aff_len(short_dim)/2  );
    std::cout << "this tf is fine\n";
  }else{
    aff_to_corner.translation() = Eigen::Vector3d( 0 , 0 ,  aff_len(short_dim)/2  );
    aff_to_corner.rotate( euler_to_quat( 180*M_PI/180, 0, 0*M_PI/180 ) );
    std::cout << "need other tf\n";
  }
  aff_to_actualpalm = aff_to_actualpalm*aff_to_corner;
  Isometry3dTime world_to_actualpalmT(0, world_to_aff_* aff_to_actualpalm);
  pc_vis_->pose_to_lcm_from_list(60003, world_to_actualpalmT );   
  
  
  // at this stage aff_to_actualpalm is a frame:
  // - position on the edge we want to grasp 
  // - (z-axis) the fingers face in the direction we want to face (including flip in each direction)
  // - (y-axis) is pointing towards knuckle on thump
  // - x-axis pointing into the face (and long the finger)
  // This is the reference point. All thats needed next is a transform back from this point for the
  // palm frame for each hand
  
  // Add on the transform from the "actual palm" to the palm link
  Eigen::Isometry3d actualplam_to_palmgeometry = Eigen::Isometry3d::Identity();
  
  // hand specific:
  if (grasp_opt_msg_.grasp_type ==0){ // Sandia left
    actualplam_to_palmgeometry.translation()  <<  -0.053 , 0.02 , -0.05  ;   // + x + y + z
    actualplam_to_palmgeometry.rotate( euler_to_quat( -20*M_PI/180, 0*M_PI/180, 180*M_PI/180 ) );
  }else{
    actualplam_to_palmgeometry.translation()  <<  -0.053 , -0.02 , -0.05  ;   // + x + y + z
    actualplam_to_palmgeometry.rotate( euler_to_quat( 20*M_PI/180, 0*M_PI/180, 180*M_PI/180 ) );
  }
  
  Eigen::Isometry3d aff_to_palmgeometry = aff_to_actualpalm*actualplam_to_palmgeometry;
  
  
  std::vector <Isometry3dTime> world_to_palmT;
  world_to_palmT.push_back( Isometry3dTime(0, world_to_aff_* aff_to_palmgeometry));
  pc_vis_->pose_collection_to_lcm_from_list(60002, world_to_palmT);   
  
  sendCandidateGrasp(aff_to_palmgeometry, 0);  
}
  
void Pass::planGraspSteeringCylinder(Eigen::Isometry3d init_grasp_pose){  
  
  // 1. Determine the Parameters of the Cylinder we need:
  // Radius and Relative Angle  
  world_to_aff_ = affutils_.getPose(aff_.origin_xyz, aff_.origin_rpy);
  Eigen::Affine3d Aff = Eigen::Affine3d(world_to_aff_.inverse() );
  Eigen::Vector3d pt = Eigen::Vector3d(init_grasp_pose.translation().x(), 
                                       init_grasp_pose.translation().y(),
                                       init_grasp_pose.translation().z());
  pt = Aff * pt;
  double rel_angle = atan2(pt(1), pt(0)  );
  double grasp_radius = sqrt(pow( pt(1),2) + pow( pt(0), 2)); // the distance of the hit point from the center of the str cyl
  std::cout << "grasp_radius: " << grasp_radius << "\n";
  
  std::map<string,double> am;
  for (int j=0; j< aff_.nparams; j++){
    am[ aff_.param_names[j] ] = aff_.params[j];
  }
  double radius = am.find("radius")->second;
  
  std::cout << rel_angle*180/M_PI << " Relative Angle around Cylinder\n";
  std::cout <<  radius << " Radius\n";
  
  double direction_yaw =0;
  double direction_offset=1;
  if (pt(2) < 0){
    std::cout << "back side (typical)\n"; // fingers pointing in +z axis
    direction_yaw =0;
    direction_offset = 1;
  }else{
    std::cout << "front side\n"; // fingers pointing in -z axis
    direction_yaw = 180;
    direction_offset = -1;
  }
  
  
  // 2. Create a reasonable afforance to hand pose 
  Eigen::Isometry3d aff_to_palmgeometry = Eigen::Isometry3d::Identity();
  // translation: outwards, upwards, forwards  
  if (grasp_opt_msg_.grasp_type ==0){ // sandia left
    // Sandia
    // was: 0.05 + radius ,0,-0.12;
    // 0.03 was too little
    aff_to_palmgeometry.rotate( euler_to_quat(direction_yaw*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
    aff_to_palmgeometry.translation()  << 0.05 + radius ,0,direction_offset*-0.10;
    aff_to_palmgeometry.rotate( euler_to_quat(-15*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
  }else if(grasp_opt_msg_.grasp_type ==1){ // sandia right
    aff_to_palmgeometry.rotate( euler_to_quat(direction_yaw*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
    aff_to_palmgeometry.translation()  << 0.05 + radius ,0,direction_offset*-0.10;
    aff_to_palmgeometry.rotate( euler_to_quat( 15*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
  }else if(grasp_opt_msg_.grasp_type ==3){ // iRobot left
    aff_to_palmgeometry.translation()  << 0.10 + radius ,0,0;
    aff_to_palmgeometry.rotate( euler_to_quat(0 *M_PI/180  , 0*M_PI/180 , 0*M_PI/180  ) );   
    if (direction_offset==-1){
      aff_to_palmgeometry.rotate( euler_to_quat(180 *M_PI/180  , 0*M_PI/180 , 0*M_PI/180  ) ); 
    }
    
  }else if(grasp_opt_msg_.grasp_type ==4){ // iRobot right
    aff_to_palmgeometry.translation()  << 0.10 + radius ,0,0;
    aff_to_palmgeometry.rotate( euler_to_quat(0 *M_PI/180  , 0*M_PI/180 , 0*M_PI/180  ) );   
    
    if (direction_offset==1){
      aff_to_palmgeometry.rotate( euler_to_quat(180 *M_PI/180  , 0*M_PI/180 , 0*M_PI/180  ) ); 
    }

  }
  
  if ( grasp_radius < 0.05 ){
    if ((grasp_opt_msg_.grasp_type ==3) || (grasp_opt_msg_.grasp_type ==4)) { 
      aff_to_palmgeometry.setIdentity();
      if (direction_offset==-1){
        aff_to_palmgeometry.rotate( euler_to_quat(0 *M_PI/180  , -90*M_PI/180 , 0*M_PI/180  ) ); 
        aff_to_palmgeometry.translation()  << 0,0,0.135;
        rel_angle =0;
      }else{
        aff_to_palmgeometry.rotate( euler_to_quat(0 *M_PI/180  , 90*M_PI/180 , 0*M_PI/180  ) ); 
        aff_to_palmgeometry.translation()  << 0,0,-0.135;
        rel_angle =0;
      }
    }
  }
    
  

  sendCandidateGrasp(aff_to_palmgeometry, rel_angle);
    
  /*
  }else if (mode_ == "plan"){
    sendPlanEELoci(aff_to_palmgeometry, {rel_angle,rel_angle +0.1,rel_angle +0.2,rel_angle +0.3,rel_angle +0.4,rel_angle +0.5,rel_angle +0.6,rel_angle +0.7,rel_angle +0.8,rel_angle +0.9,rel_angle +1.0} );
  }else if (mode_ == "both"){
    sendCandidateGrasp(aff_to_palmgeometry, rel_angle);
    sendPlanEELoci(aff_to_palmgeometry, {rel_angle,rel_angle +0.1,rel_angle +0.2,rel_angle +0.3,rel_angle +0.4,rel_angle +0.5,rel_angle +0.6,rel_angle +0.7,rel_angle +0.8,rel_angle +0.9,rel_angle +1.0} );
  }
  */
}


drc::position_3d_t EigenToDRC(Eigen::Isometry3d pose){
  drc::position_3d_t hand_pose;
  hand_pose.translation.x = pose.translation().x();
  hand_pose.translation.y = pose.translation().y();
  hand_pose.translation.z = pose.translation().z();
  Quaterniond q = Quaterniond( pose.rotation() );
  hand_pose.rotation.w = q.w();
  hand_pose.rotation.x = q.x();
  hand_pose.rotation.y = q.y();
  hand_pose.rotation.z = q.z();
  return hand_pose;
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

Eigen::Isometry3d getRotPose(double rel_angle){
  // The output is the relative pose of the end effector with the radius and rel angle:
  Eigen::Isometry3d rot_pose = Eigen::Isometry3d::Identity();
  rot_pose.translation()  << 0,0,0;
  rot_pose.rotate( euler_to_quat(0 *M_PI/180  , 0*M_PI/180 , rel_angle ) ); 
  return rot_pose;
}


// hand_pose is in the affordance's frame
void Pass::sendCandidateGrasp(Eigen::Isometry3d aff_to_palmgeometry, double rel_angle){
  Eigen::Isometry3d pose = getRotPose(rel_angle)*aff_to_palmgeometry;
  drc::position_3d_t hand_pose = EigenToDRC(pose);
  
  drc::desired_grasp_state_t cg;
  cg.utime = grasp_opt_msg_.utime;
  cg.robot_name =grasp_opt_msg_.robot_name;
  cg.object_name =grasp_opt_msg_.object_name;
  cg.geometry_name = grasp_opt_msg_.geometry_name;
  cg.unique_id = grasp_opt_msg_.unique_id;
  cg.grasp_type = grasp_opt_msg_.grasp_type;
  cg.power_grasp = false;
  if ((grasp_opt_msg_.grasp_type ==0) || (grasp_opt_msg_.grasp_type ==3) ){ // if sandia left or irobot left
    cg.l_hand_pose = hand_pose;
  }else {
    cg.r_hand_pose = hand_pose;
  }
  
  if ( (grasp_opt_msg_.grasp_type ==0) || (grasp_opt_msg_.grasp_type ==1) ){ // sandia
    cg.l_joint_name = sandia_l_joint_name_;
    cg.l_joint_position = sandia_l_joint_position_;
    cg.r_joint_name = sandia_r_joint_name_;
    cg.r_joint_position = sandia_r_joint_position_;
  }else if ( (grasp_opt_msg_.grasp_type ==3) || (grasp_opt_msg_.grasp_type ==4) ){ // irobot
    cg.l_joint_name = irobot_l_joint_name_;
    cg.l_joint_position = irobot_l_joint_position_;
    cg.r_joint_name = irobot_r_joint_name_;
    cg.r_joint_position = irobot_r_joint_position_;
  }else if ( (grasp_opt_msg_.grasp_type ==6) || (grasp_opt_msg_.grasp_type ==7) ){ // robotiq
    cg.l_joint_name = robotiq_l_joint_name_;
    cg.l_joint_position = robotiq_l_joint_position_;
    cg.r_joint_name = robotiq_r_joint_name_;
    cg.r_joint_position = robotiq_r_joint_position_;
  }
  
  cg.num_l_joints =cg.l_joint_position.size();
  cg.num_r_joints =cg.r_joint_position.size();

  lcm_->publish("CANDIDATE_GRASP", &cg);  
}


int main(int argc, char ** argv) {
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
