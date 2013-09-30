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
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, 
         std::string mode_, bool use_irobot_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;        
    std::string mode_;
    bool use_irobot_;
    bool aff_ready_;
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


    void sendCandidateGrasp(const  drc::grasp_opt_control_t* msg, 
                            Eigen::Isometry3d aff_to_palmgeometry, double rel_angle);
    void sendPlanEELoci(const  drc::grasp_opt_control_t* msg, 
                        Eigen::Isometry3d aff_to_palmgeometry, std::vector <double> rel_angles);

    void sendStandingPosition();
    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;
    
    
    drc::affordance_t aff_;
    Eigen::Isometry3d world_to_aff_ ;
    Eigen::Isometry3d world_to_body_;
    double ground_height_;
    
    AffordanceUtils affutils_;
    
    vector<string> l_joint_name_;
    vector<string> r_joint_name_;
    vector<double> l_joint_position_;    
    vector<double> r_joint_position_;
    
    std::vector <Isometry3dTime> eeloci_poses_;
    bool eeloci_plan_outstanding_;    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string mode_, bool use_irobot_):
    lcm_(lcm_), mode_(mode_), use_irobot_(use_irobot_){
  aff_ready_ = false;
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
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60001,"Grasp Frames",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60012,"Grasp Pose Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60013,"Grasp Feasibility" ,1,1, 60012,0, { 0.0, 1.0, 0.0} ));
  //
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60014,"Standing Position",4,1) ); // 4 is pose3d
      

  l_joint_name_ = {"left_f0_j0","left_f0_j1","left_f0_j2",
  "left_f1_j0","left_f1_j1","left_f1_j2",
  "left_f2_j0","left_f2_j1","left_f2_j2",
  "left_f3_j0","left_f3_j1","left_f3_j2" };
  
  l_joint_position_ = {0,0,0,  0,0,0,
                       0,0,0,  0,0,0};

  r_joint_name_ = {"right_f0_j0","right_f0_j1","right_f0_j2",
  "right_f1_j0","right_f1_j1","right_f1_j2",
  "right_f2_j0","right_f2_j1","right_f2_j2",
  "right_f3_j0","right_f3_j1","right_f3_j2" };
  
  // fore, middle, little, thumb | lower mid upper
  r_joint_position_ = {0,0,0,  0,0,0,
                       0,0,0,  0,0,0};
//  r_joint_position_ = {0,0.7,0.7,  0,0.7,0.7,
                       //0,0.7,0.7,  -0.15,0.68,0.47};  
                       
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


void Pass::sendStandingPosition(){ 
      int counter = 0;


      std::vector<Isometry3dTime>  feet_positionsT;

      for (int front_side=0; front_side < 2; front_side++){
      
      Eigen::Isometry3d valve_pose(Eigen::Isometry3d::Identity());
      valve_pose.translation()  << aff_.origin_xyz[0], aff_.origin_xyz[1], ground_height_;
      if (front_side){
        valve_pose.rotate( Eigen::Quaterniond(  euler_to_quat( 0 ,  0 ,  aff_.origin_rpy[2] - 90*M_PI/180 )  ) );
      }else{
        valve_pose.rotate( Eigen::Quaterniond(  euler_to_quat( 0 ,  0 ,  aff_.origin_rpy[2] + 90*M_PI/180 )  ) );
      }
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose) );


      for (int left_reach =0 ; left_reach<2 ; left_reach++){

      Eigen::Isometry3d valve2com(Eigen::Isometry3d::Identity());
      if (left_reach){
        valve2com.translation()  << -0.45, 0.28, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,-20*M_PI/180))  );   
      }else{
        valve2com.translation()  << -0.45, -0.28, 0;
        valve2com.rotate( Eigen::Quaterniond(euler_to_quat(0,0,20*M_PI/180))  );   
      }
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com) );

      Eigen::Isometry3d com2left(Eigen::Isometry3d::Identity());
      com2left.translation()  << 0.0, 0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2left) );
      Eigen::Isometry3d com2right(Eigen::Isometry3d::Identity());
      com2right.translation()  << 0.0, -0.13, 0;
      feet_positionsT.push_back( Isometry3dTime(counter++, valve_pose*valve2com*com2right) );


      }
      }

      pc_vis_->pose_collection_to_lcm_from_list(60014, feet_positionsT); 
}


void Pass::affHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  std::cout << "got "<< msg->naffs << " affs\n";
  if (msg->naffs > 0){
    aff_ = msg->affs_plus[0].aff;
    aff_ready_ = true;

    bool send_standing_position_ =true;
    if (send_standing_position_){
      sendStandingPosition();

    }
  }else{
    aff_ready_ = false; 
  }
}


void Pass::planHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg){
  if (!eeloci_plan_outstanding_){
    return;
  }
  std::cout << "got a plan\n";
  
  if (eeloci_poses_.size() != msg->num_states ){
    std::cout << eeloci_poses_.size() <<" n. loci is not number of plan states ["<<msg->num_states<<"]. will need to solve FK\n"; 
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud->width   = msg->num_states;
  cloud->height   = 1;
  cloud->points.resize (msg->num_states);
  for (size_t i = 0; i < msg->num_states; i++) {
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


void Pass::initGraspHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::grasp_opt_control_t* msg){
  if ( (!aff_ready_) || (!cartpos_ready_) ){
    std::cout << "Affordances or State Not received\n";
    return; 
  }
  
  std::vector<Isometry3dTime> grasp_poseT;
  Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
  grasp_pose.translation() << msg->l_hand_init_pose.translation.x, msg->l_hand_init_pose.translation.y, msg->l_hand_init_pose.translation.z;
  grasp_poseT.push_back( Isometry3dTime(msg->utime, grasp_pose) );
  pc_vis_->pose_collection_to_lcm_from_list(60011, grasp_poseT); 
  
  
  // 1. Determine the Parameters of the Cylinder we need:
  // Radius and Relative Angle  
  world_to_aff_ = affutils_.getPose(aff_.origin_xyz, aff_.origin_rpy);
  Eigen::Affine3d Aff = Eigen::Affine3d(world_to_aff_.inverse() );
  Eigen::Vector3d pt = Eigen::Vector3d(msg->l_hand_init_pose.translation.x, 
                                       msg->l_hand_init_pose.translation.y,
                                       msg->l_hand_init_pose.translation.z);
  pt = Aff * pt;
  double rel_angle = atan2(pt(1), pt(0)  );
  
  std::map<string,double> am;
  for (size_t j=0; j< aff_.nparams; j++){
    am[ aff_.param_names[j] ] = aff_.params[j];
  }
  double radius = am.find("radius")->second;
  
  std::cout << rel_angle*180/M_PI << " Relative Angle around Cylinder\n";
  std::cout <<  radius << " Radius\n";
  
  
  // 2. Create a reasonable afforance to hand pose 
  Eigen::Isometry3d aff_to_palmgeometry = Eigen::Isometry3d::Identity();
  // translation: outwards, upwards, forwards  
  if (use_irobot_){ // iRobot
    if (msg->grasp_type ==0){ // iRobot left
      aff_to_palmgeometry.translation()  << 0.01 + radius ,0,-0.09;
      aff_to_palmgeometry.rotate( euler_to_quat(0 *M_PI/180  , 0*M_PI/180 , 0*M_PI/180  ) );   
    }else{ // iRobot right
      aff_to_palmgeometry.translation()  << 0.01 + radius ,0,-0.09;
      aff_to_palmgeometry.rotate( euler_to_quat(0 *M_PI/180  , 0*M_PI/180 , 0*M_PI/180  ) );   
    }
  }else{ // Sandia
    // was: 0.05 + radius ,0,-0.12;
    // 0.03 was too little
    if (msg->grasp_type ==0){ // sandia left
      aff_to_palmgeometry.translation()  << 0.05 + radius ,0,-0.10;
      aff_to_palmgeometry.rotate( euler_to_quat(-15*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
    }else{ // sandia right
      aff_to_palmgeometry.translation()  << 0.05 + radius ,0,-0.10;
      aff_to_palmgeometry.rotate( euler_to_quat( 15*M_PI/180, 0*M_PI/180, 0*M_PI/180  ) );   
    }
    
    
  }
  

  if (mode_ == "grasp" ){
    sendCandidateGrasp(msg,aff_to_palmgeometry, rel_angle);
  }else if (mode_ == "plan"){
    sendPlanEELoci(msg,aff_to_palmgeometry, {rel_angle,rel_angle +0.1,rel_angle +0.2,rel_angle +0.3,rel_angle +0.4,rel_angle +0.5,rel_angle +0.6,rel_angle +0.7,rel_angle +0.8,rel_angle +0.9,rel_angle +1.0} );
  }else if (mode_ == "both"){
    sendCandidateGrasp(msg,aff_to_palmgeometry, rel_angle);
    sendPlanEELoci(msg,aff_to_palmgeometry, {rel_angle,rel_angle +0.1,rel_angle +0.2,rel_angle +0.3,rel_angle +0.4,rel_angle +0.5,rel_angle +0.6,rel_angle +0.7,rel_angle +0.8,rel_angle +0.9,rel_angle +1.0} );

    /*
    std::vector <double> rel_angles;
    for (double i= 0; i< M_PI*2 ; i=i+M_PI/10){
      rel_angles.push_back(rel_angle + i);
    }
    sendPlanEELoci(msg,aff_to_palmgeometry, rel_angles );
    */
    
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


drc::position_3d_t EigenToDRC(Eigen::Isometry3d &pose){
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
void Pass::sendCandidateGrasp(const  drc::grasp_opt_control_t* msg, Eigen::Isometry3d aff_to_palmgeometry, double rel_angle){
  Eigen::Isometry3d pose = getRotPose(rel_angle)*aff_to_palmgeometry;
  drc::position_3d_t hand_pose = EigenToDRC(pose);
  
  drc::desired_grasp_state_t cg;
  cg.utime = msg->utime;
  cg.robot_name =msg->robot_name;
  cg.object_name =msg->object_name;
  cg.geometry_name = msg->geometry_name;
  cg.unique_id = msg->unique_id;
  cg.grasp_type = msg->grasp_type;
  cg.power_grasp = false;
  if (msg->grasp_type ==0){
    cg.l_hand_pose = hand_pose;
  }else {
    cg.r_hand_pose = hand_pose;
  }
  
  cg.l_joint_name = l_joint_name_;
  cg.l_joint_position = l_joint_position_;
  cg.r_joint_name = r_joint_name_;
  cg.r_joint_position = r_joint_position_;
  cg.num_l_joints =l_joint_name_.size();
  cg.num_r_joints =r_joint_name_.size();

  lcm_->publish("CANDIDATE_GRASP", &cg);  
}




void Pass::sendPlanEELoci(const  drc::grasp_opt_control_t* msg, Eigen::Isometry3d aff_to_palmgeometry, std::vector <double> rel_angles){
  drc::traj_opt_constraint_t traj;
  traj.utime =0;
  traj.robot_name ="atlas";
  
  // Unresolved transformation between palmgeometry and palm used by Sisir in grasping:
  // TODO: correctly determine this TF!
  Eigen::Isometry3d palmgeometry_to_palm = Eigen::Isometry3d::Identity();
  palmgeometry_to_palm.translation()  << 0,0,0;
  palmgeometry_to_palm.rotate( euler_to_quat(0, -90*M_PI/180, 0) );    
  

  // This fk stuff is not used:
  //Eigen::Isometry3d body_to_left_palm = KDLToEigen(cartpos_.find("left_palm")->second);
  //Eigen::Isometry3d body_to_l_hand = KDLToEigen(cartpos_.find("l_hand")->second);
  //Eigen::Isometry3d l_hand_to_left_palm = body_to_l_hand.inverse() * body_to_left_palm ;
    
  
  eeloci_poses_.clear();

  for (size_t i=0;i < rel_angles.size(); i++){
    Eigen::Isometry3d pose =  world_to_aff_ * getRotPose(rel_angles[i])*aff_to_palmgeometry * palmgeometry_to_palm;// * l_hand_to_left_palm;
    drc::position_3d_t hand_pose = EigenToDRC(pose);
    
    traj.link_origin_position.push_back(hand_pose);
    traj.link_timestamps.push_back( i*1E6);
    
    if (msg->grasp_type ==0){
      traj.link_name.push_back("left_palm");
    }else {
      traj.link_name.push_back("right_palm");
    }
    
    eeloci_poses_.push_back( Isometry3dTime(i, pose));//*palmgeometry_to_palm) );
  }
  traj.num_links = rel_angles.size();
  
  if (msg->grasp_type ==0){
    traj.joint_name = l_joint_name_;
    traj.joint_position = l_joint_position_;
  }else{
    traj.joint_name = r_joint_name_;
    traj.joint_position = r_joint_position_;
  }
  traj.num_joints =traj.joint_name.size();
  traj.joint_timestamps.assign( traj.num_joints ,0);
  
  lcm_->publish("DESIRED_MANIP_PLAN_EE_LOCI", &traj);
  pc_vis_->pose_collection_to_lcm_from_list(60001, eeloci_poses_); 
  eeloci_plan_outstanding_ = true;

}


int main(int argc, char ** argv) {
  string mode = "both";
  bool use_irobot = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(mode, "m", "mode","mode: both, grasp, plan");
  opt.add(use_irobot, "i", "use_irobot","iRobot hand (otherwise Sandia)");
  opt.parse();
  std::cout << "mode: " << mode << "\n";  
  std::cout << "use_irobot: " << use_irobot << "\n";  

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,mode,use_irobot);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
