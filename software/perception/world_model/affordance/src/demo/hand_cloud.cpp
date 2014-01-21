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
#include "lcmtypes/drc/affordance_plus_t.hpp"
#include "lcmtypes/drc/affordance_plus_collection_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include <lcmtypes/bot_core.hpp>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

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
    bool cartpos_ready_;
    
    void affHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::affordance_plus_collection_t* msg);      
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);
    
    drc::affordance_plus_t  getUtorsoAffordance( Eigen::Isometry3d &pose);

    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;
    
    AffordanceUtils affutils_;
    
    int utorso_irobot_affordance_uid_, utorso_sandia_affordance_uid_;
    int64_t last_update_utime_;// last time the affordance was updated
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){
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
  
  utorso_irobot_affordance_uid_ =-1; // havent found the id yet
  utorso_sandia_affordance_uid_ =-1; // havent found the id yet
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


void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (msg->utime - last_update_utime_ < 1E6){
    // std::cout << "won't update for one second\n";
    return; 
  }
  
  last_update_utime_ = msg->utime;
  
  // 0. Extract World Pose of body:
  Eigen::Isometry3d world_to_body;
  world_to_body.setIdentity();
  world_to_body.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body.rotate(quat);    
    
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
  Eigen::Isometry3d world_to_utorso = world_to_body* body_to_utorso;
  drc::affordance_plus_t a1 = getUtorsoAffordance( world_to_utorso );
  a1.aff.aff_store_control = drc::affordance_t::UPDATE;
  if (utorso_irobot_affordance_uid_!=-1){
    a1.aff.uid = utorso_irobot_affordance_uid_;
    a1.aff.otdf_type = "utorso_irobot";
    lcm_->publish("AFFORDANCE_TRACK",&a1.aff);  
    std::cout << "Updating utorso irobot affordance\n";  
  }
  if (utorso_sandia_affordance_uid_!=-1){
    a1.aff.uid = utorso_sandia_affordance_uid_;
    a1.aff.otdf_type = "utorso_sandia";
    lcm_->publish("AFFORDANCE_TRACK",&a1.aff); 
    std::cout << "Updating utorso sandia affordance\n";   
  }


  return;
}


drc::affordance_plus_t Pass::getUtorsoAffordance(Eigen::Isometry3d &pose){
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =1;
    a.otdf_type ="put_type_here";
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
  utorso_irobot_affordance_uid_ =-1;
  utorso_sandia_affordance_uid_ =-1;
    
  for (int i=0 ; i < msg->naffs ; i++){
    drc::affordance_t aff = msg->affs_plus[i].aff;
    
    if (aff.otdf_type == "utorso_irobot" ){
      std::cout << aff.uid << " was found - the utorso_irobot aff\n";
      utorso_irobot_affordance_uid_ = aff.uid;
    }

    if (aff.otdf_type == "utorso_sandia" ){
      std::cout << aff.uid << " was found - the utorso_sandia aff\n";
      utorso_sandia_affordance_uid_ = aff.uid;
    }
  }
}

int main(int argc, char ** argv) {
/*  bool use_irobot = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(use_right, "r", "use_right","Right hand (otherwise left)");
  opt.parse();
  std::cout << "use_right: " << use_right << "\n";  
*/

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
