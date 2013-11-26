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


struct Config{
  
  bool use_sandia_;
  bool use_left_hand_;
  
  Config () {
        use_sandia_ = false;
        use_left_hand_ = false;
  }
};


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, Config& config_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    Config config_;
    pointcloud_vis* pc_vis_;  
    bool cartpos_ready_;
    
    void affHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::affordance_plus_collection_t* msg);      
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);


    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;
    drc::robot_state_t rstate_;
    bool rstate_init_;
    
    
    // Affordances stored using the object_name that Sisir seems to be sending with INIT_GRASP_OPT_* messages
    // [otdf_type]_[uid]
    map<string, drc::affordance_t > affs_;
    
    drc::affordance_t aff_;
    Eigen::Isometry3d world_to_aff_ ;
    Eigen::Isometry3d world_to_body_;
    double ground_height_;
    
    AffordanceUtils affutils_;
    
    std::vector <Isometry3dTime> eeloci_poses_;
    bool eeloci_plan_outstanding_;    
    
    drc::grasp_opt_control_t grasp_opt_msg_;

    void solveFK(drc::robot_state_t state, Eigen::Isometry3d &world_to_body, 
                 map<string, KDL::Frame > &cartpos, bool &cartpos_ready  );    
    std::string getPalmLink();
    int getHandType();
    
    void publishCandidateGrasp(Eigen::Isometry3d aff_to_hand, 
                                 string object_name, int hand_type);
    
    vector<string> sandia_l_joint_name_;
    vector<string> sandia_r_joint_name_;
    vector<double> sandia_l_joint_position_;    
    vector<double> sandia_r_joint_position_;
    
    vector<string> irobot_l_joint_name_;
    vector<string> irobot_r_joint_name_;
    vector<double> irobot_l_joint_position_;    
    vector<double> irobot_r_joint_position_;    
    
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, Config& config_):
    lcm_(lcm_),config_(config_){
  cartpos_ready_ = false;
      
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = new KDL::TreeFkSolverPosFull_recursive(tree);
      
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robot_state_handler,this);  
  
  lcm_->subscribe( "AFFORDANCE_PLUS_COLLECTION" ,&Pass::affHandler,this);

  
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
  
  
  eeloci_plan_outstanding_ = false;
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


void Pass::solveFK(drc::robot_state_t state, Eigen::Isometry3d &world_to_body, map<string, KDL::Frame > &cartpos, bool &cartpos_ready  ){
  // 0. Extract World Pose of body:
  world_to_body.setIdentity();
  world_to_body.translation()  << state.pose.translation.x, state.pose.translation.y, state.pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(state.pose.rotation.w, state.pose.rotation.x, 
                                               state.pose.rotation.y, state.pose.rotation.z);
  world_to_body.rotate(quat);    
    
  // 1. Solve for Forward Kinematics:
  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  cartpos.clear();
  for (uint i=0; i< (uint) state.num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(state.joint_name[i], state.joint_position[i]));
  
  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  kinematics_status = fksolver_->JntToCart(jointpos_in,cartpos,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  cartpos_ready=true;  
}




std::string Pass::getPalmLink(){
  std::string palm_link = "left_palm";
  if (config_.use_left_hand_ && config_.use_sandia_){
    palm_link = "left_palm";
  }else if (!config_.use_left_hand_ && config_.use_sandia_){
    palm_link = "right_palm";
  }else if (config_.use_left_hand_ && !config_.use_sandia_){
    palm_link = "left_base_link";
  }else if (!config_.use_left_hand_ && !config_.use_sandia_){
    palm_link = "right_base_link";
  }  
  return palm_link;
}

int Pass::getHandType(){
  int hand_type = 0;
  if (config_.use_left_hand_ && config_.use_sandia_){
    hand_type = 0;
  }else if (!config_.use_left_hand_ && config_.use_sandia_){
    hand_type = 1;
  }else if (config_.use_left_hand_ && !config_.use_sandia_){
    hand_type = 3;
  }else if (!config_.use_left_hand_ && !config_.use_sandia_){
    hand_type = 4;
  }  
  return hand_type;
}

void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  rstate_= *msg;
  rstate_init_ = true;
  return;
}




void Pass::publishCandidateGrasp(Eigen::Isometry3d aff_to_hand, 
                                 string object_name, int hand_type){
  drc::position_3d_t hand_position = EigenToDRC(aff_to_hand);
  drc::desired_grasp_state_t cg;
  cg.utime = bot_timestamp_now();
  cg.robot_name = "atlas";
  cg.object_name =object_name;// object_name ;
  cg.geometry_name = "cylinder_0_0"; // main collar
  cg.unique_id = 22;
  cg.grasp_type = hand_type;
  cg.power_grasp =false;

  if (hand_type ==3){
    cg.l_hand_pose = hand_position;
    cg.r_hand_pose = EigenToDRC(Eigen::Isometry3d::Identity());
  }else {
    cg.l_hand_pose = EigenToDRC(Eigen::Isometry3d::Identity());
    cg.r_hand_pose = hand_position;
  }
  cg.l_joint_name = irobot_l_joint_name_;
  cg.l_joint_position = irobot_l_joint_position_;
  cg.r_joint_name = irobot_r_joint_name_;
  cg.r_joint_position = irobot_r_joint_position_;
  cg.num_l_joints =cg.l_joint_position.size();
  cg.num_r_joints =cg.r_joint_position.size();
  lcm_->publish("CANDIDATE_GRASP", &cg);    
}



void Pass::affHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  //std::cout << "got "<< msg->naffs << " affs\n";
  if(rstate_init_){
      solveFK(rstate_,world_to_body_, cartpos_, cartpos_ready_);
  }else{
    return;
  }
    
  affs_.clear();
  for (int i=0 ; i < msg->naffs ; i++){
    drc::affordance_t aff = msg->affs_plus[i].aff;
    std::stringstream ss;
    ss << aff.otdf_type << '_' << aff.uid;    
    // std::cout << ss.str() << "\n";

    affs_[ ss.str() ]=  aff;
    
    if (aff.otdf_type == "firehose"){
      
      
      Eigen::Isometry3d body_to_palm = KDLToEigen(cartpos_.find( getPalmLink() )->second);
      Eigen::Isometry3d world_to_palm =  world_to_body_* body_to_palm;
      
      Eigen::Isometry3d palm_to_hose = Eigen::Isometry3d::Identity();
      palm_to_hose.translation()<< 0,0,0.115;
      palm_to_hose.rotate( euler_to_quat ( 0 , M_PI/2, 0  ) );
      Eigen::Isometry3d world_to_hose =  world_to_palm* palm_to_hose;
      
      
      
      affutils_.setXYZRPYFromIsometry3d(aff.origin_xyz, aff.origin_rpy, world_to_hose);
  
      aff.aff_store_control = drc::affordance_t::UPDATE;
      lcm_->publish("AFFORDANCE_TRACK", &aff);  

      std::stringstream ss;
      ss << aff.otdf_type << '_' << aff.uid;    
      
      Eigen::Isometry3d hose_to_palm = Eigen::Isometry3d::Identity();
      hose_to_palm.translation()<< 0.115,0,0;
      hose_to_palm.rotate( euler_to_quat ( -M_PI/2 , 0, 0  ) );
     
      
      
      publishCandidateGrasp(hose_to_palm, ss.str() ,  getHandType() );
      
      
      //sendStandingPositionValve( aff );
    }
  }    
}



int main(int argc, char ** argv) {
  Config config;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(config.use_left_hand_, "l", "use_left_hand","use left hand [defualt is right]");
  opt.parse();
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, config);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
