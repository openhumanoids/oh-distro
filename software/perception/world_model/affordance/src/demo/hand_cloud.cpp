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
         bool use_irobot_, bool use_right_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;     
    bool use_irobot_;
    bool use_right_;
    bool cartpos_ready_;
    
    void affHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::affordance_plus_collection_t* msg);      
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);
    
    drc::affordance_plus_t  getWorldAffordance( Eigen::Isometry3d pose);
    void sendCandidateGrasp(Eigen::Isometry3d pose,string object_name, string geometry_name, int unique_id, int grasp_type );

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
    
    drc::grasp_opt_control_t grasp_opt_msg_;
    
    int world_affordance_uid_;
    int64_t last_update_utime_;// last time the hands were updated
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_,  bool use_irobot_, bool use_right_):
    lcm_(lcm_),  use_irobot_(use_irobot_), use_right_(use_right_){
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
  pc_vis_->obj_cfg_list.push_back( obj_cfg(7001,"Hands",5,1) );
  
  
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
  
  world_affordance_uid_ =-1; // havent found the id yet
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


void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (world_affordance_uid_==-1){
    std::cout << "no world aff yet\n";
    return;
  }
  if (msg->utime - last_update_utime_ < 1E6){
    std::cout << "won't update for one second\n";
    return; 
  }
  
  last_update_utime_ = msg->utime;
  
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

  
  Eigen::Isometry3d body_to_utorso = KDLToEigen(cartpos_.find("utorso")->second);
  drc::affordance_plus_t a1 = getWorldAffordance( world_to_body_* body_to_utorso );
  a1.aff.uid = world_affordance_uid_;
  a1.aff.aff_store_control = drc::affordance_t::UPDATE;
  lcm_->publish("AFFORDANCE_TRACK",&a1.aff);  
  
  return;
  
  // whats below this is iunnecassary:
  /*
  
  int grasp_type = 0; // 0 left sandia | 
  if (!use_right_ && !use_irobot_){
    grasp_type = 0; // left sandia hand
  }else if (use_right_ && !use_irobot_){
    grasp_type = 1; // right sandia hand
  }else if (!use_right_ && use_irobot_){
    grasp_type = 3; // left irobot hand
  }else if (use_right_ && use_irobot_){
    grasp_type = 4; // right irobot hand
  }  
  
  
  
  
  Eigen::Isometry3d utorso_to_palmframe;
  utorso_to_palmframe.setIdentity();
  
  std::vector < std::vector<double> > hand_poses;
  
  hand_poses.push_back( {0.5,0,0.5,0,0,-0.7855}  );

  
  ///
  std::vector<double> pose = hand_poses[0];
  utorso_to_palmframe.translation()  <<  pose[0], pose[1], pose[2];
  utorso_to_palmframe.rotate ( euler_to_quat(pose[3] , pose[4] , pose[5]  ) );
  
  

    
  
  /// Add Rotation of palm frame onto the look direction:  
  if (grasp_type==0){
    utorso_to_palmframe.rotate ( euler_to_quat(M_PI/2,0,M_PI/2) ); //left  sandia hand facing forward and thumb right
  }else if (grasp_type==3){ 
    utorso_to_palmframe.rotate ( euler_to_quat(-M_PI/2,M_PI,0) ); // left irobot hand facing forward single finger right
  }
  
  
  Eigen::Isometry3d  world_to_palmframe = world_to_body_* body_to_utorso * utorso_to_palmframe ;
  
  std::vector<Isometry3dTime> world_to_palmframeT;
  world_to_palmframeT.push_back(  Isometry3dTime(0,world_to_palmframe) );
  
  pc_vis_->pose_collection_to_lcm_from_list(7001, world_to_palmframeT);
 
  string object_name = "world_1";
  string geometry_name = "box_0"; // from the otdf directly
  
  int unique_id =0;
  
  sendCandidateGrasp( world_to_palmframe, object_name, geometry_name, unique_id, grasp_type );
 */ 
}





// hand_pose is in the affordance's frame
void Pass::sendCandidateGrasp(Eigen::Isometry3d pose,string object_name, string geometry_name, int unique_id, int grasp_type ){
  drc::position_3d_t hand_pose = EigenToDRC(pose);
  
  drc::desired_grasp_state_t cg;
  cg.utime = 0; //bot_timestamp_now();
  cg.robot_name ="atlas";
  cg.object_name =object_name;
  cg.geometry_name = geometry_name;
  cg.unique_id = unique_id;
  cg.grasp_type = grasp_type;
  cg.power_grasp = false;
  cg.l_hand_pose = hand_pose;
  
  if ( (grasp_type ==0) || (grasp_type ==1) ){ // sandia
    cg.l_joint_name = sandia_l_joint_name_;
    cg.l_joint_position = sandia_l_joint_position_;
    cg.r_joint_name = sandia_r_joint_name_;
    cg.r_joint_position = sandia_r_joint_position_;
  }else{
    cg.l_joint_name = irobot_l_joint_name_;
    cg.l_joint_position = irobot_l_joint_position_;
    cg.r_joint_name = irobot_r_joint_name_;
    cg.r_joint_position = irobot_r_joint_position_;
  }
  cg.num_l_joints =cg.l_joint_position.size();
  cg.num_r_joints =cg.r_joint_position.size();

  lcm_->publish("CANDIDATE_GRASP", &cg);  
}



drc::affordance_plus_t Pass::getWorldAffordance(Eigen::Isometry3d pose){
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =1;
    a.otdf_type ="world";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("lX");    a.params.push_back(0.01000);
    a.param_names.push_back("lY");    a.params.push_back(0.01000);
    a.param_names.push_back("lZ");    a.params.push_back(0.01000);
    a.param_names.push_back("mass");    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;
    
    affutils_.setXYZRPYFromIsometry3d(a.origin_xyz, a.origin_rpy, pose);
        
    drc::affordance_plus_t a1;
    a1.aff = a;
    a1.npoints=0; 
    a1.ntriangles =0; 
    
    return a1;
}


void Pass::affHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  std::cout << "got "<< msg->naffs << " affs\n";
  
  bool found_world_affordance=false;
  
  for (int i=0 ; i < msg->naffs ; i++){
    drc::affordance_t aff = msg->affs_plus[i].aff;
    
    // if its at zero and has a 1cm bounding box, then assume its the world affordance
    if (aff.otdf_type == "world" ){
      std::cout << aff.uid << " was found - the world aff\n";
      found_world_affordance=true;
      world_affordance_uid_ = aff.uid;
    }
  }    
  
  if(!found_world_affordance){
    std::cout << "will make a new aff\n";
    drc::affordance_plus_t a1 = getWorldAffordance( Eigen::Isometry3d::Identity() );
    lcm_->publish("AFFORDANCE_FIT",&a1);
  }
}




Eigen::Isometry3d getRotPose(double rel_angle){
  // The output is the relative pose of the end effector with the radius and rel angle:
  Eigen::Isometry3d rot_pose = Eigen::Isometry3d::Identity();
  rot_pose.translation()  << 0,0,0;
  rot_pose.rotate( euler_to_quat(0 *M_PI/180  , 0*M_PI/180 , rel_angle ) ); 
  return rot_pose;
}


int main(int argc, char ** argv) {
  bool use_irobot = false;
  bool use_right = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(use_irobot, "i", "use_irobot","iRobot hand (otherwise Sandia)");
  opt.add(use_right, "r", "use_right","Right hand (otherwise left)");
  opt.parse();
  std::cout << "use_irobot: " << use_irobot << "\n";
  std::cout << "use_right: " << use_right << "\n";  

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,use_irobot,use_right);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
