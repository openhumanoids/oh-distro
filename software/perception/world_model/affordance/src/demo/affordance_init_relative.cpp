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

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>


#include <pointcloud_tools/pointcloud_vis.hpp>
#include <pointcloud_tools/pointcloud_math.hpp>

#include <affordance/AffordanceUtils.hpp>

#include <ConciseArgs>
using namespace Eigen;


using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string mode_, int which_affordance_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;        
    std::string mode_;
    bool aff_ready_;
    bool cartpos_ready_;
    
    void affHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::affordance_plus_collection_t* msg);      
    void initGraspHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::grasp_opt_control_t* msg);
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);

    void sendCandidateGrasp(const  drc::grasp_opt_control_t* msg, 
                            Eigen::Isometry3d aff_to_palmgeometry, double rel_angle);
    void sendPlanEELoci(const  drc::grasp_opt_control_t* msg, 
                        Eigen::Isometry3d aff_to_palmgeometry, std::vector <double> rel_angles);

    
    drc::affordance_plus_t getSteeringCylinderAffordancePlus(std::string filename, Eigen::Isometry3d utorso_to_aff, int uid);
    drc::affordance_plus_t getCylinderAffordancePlus(std::string filename, Eigen::Isometry3d utorso_to_aff, int uid);
    drc::affordance_plus_t getBoxAffordancePlus(std::string filename, Eigen::Isometry3d utorso_to_aff, int uid);
    
    BotParam* botparam_;
    bot::frames* frames_cpp_;
    
    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;
    
    Eigen::Isometry3d world_to_utorso_;
    AffordanceUtils affutils_;
    
    // place the affordance relative to left shoulder
    int which_affordance_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string mode_, int which_affordance_):
    lcm_(lcm_), mode_(mode_), which_affordance_(which_affordance_){
  aff_ready_ = false;
  cartpos_ready_ = false;
      
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = new KDL::TreeFkSolverPosFull_recursive(tree);
      
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  frames_cpp_ = new bot::frames(lcm_ , botparam_);  
  
  sleep(1);
  
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robot_state_handler,this);  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
      

}


drc::affordance_plus_t Pass::getSteeringCylinderAffordancePlus(std::string filename, Eigen::Isometry3d utorso_to_aff, int uid){ 
  Eigen::Isometry3d world_to_aff = world_to_utorso_*utorso_to_aff; 
  std::vector<double> xyzrpy = {.0 , .0 , .0, 0. , 0 , 0};
  xyzrpy[0] = world_to_aff.translation().x();
  xyzrpy[1] = world_to_aff.translation().y();
  xyzrpy[2] = world_to_aff.translation().z();
  quat_to_euler ( Eigen::Quaterniond(world_to_aff.rotation() ), xyzrpy[3], xyzrpy[4], xyzrpy[5] );  
  
  drc::affordance_plus_t p;
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="steering_cyl";
  a.aff_store_control = drc::affordance_t::NEW;
  
  a.params.push_back(0.03  ); a.param_names.push_back("length");
  a.params.push_back(0.15  ); a.param_names.push_back("radius");
  a.params.push_back(1.0  ); a.param_names.push_back("mass");
  a.nparams =a.params.size();
  a.nstates =0;

  a.origin_xyz[0]=xyzrpy[0]; a.origin_xyz[1]=xyzrpy[1]; a.origin_xyz[2]=xyzrpy[2]; 
  a.origin_rpy[0]=xyzrpy[3]; a.origin_rpy[1]=xyzrpy[4]; a.origin_rpy[2]=xyzrpy[5]; 
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0;   
 
  p.aff = a;
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  p.npoints=0; 
  p.ntriangles =0;
  
  p.aff.bounding_lwh[0]=0.36;       p.aff.bounding_lwh[1]=0.33;      p.aff.bounding_lwh[2]=0.3;   
  return p;
}


drc::affordance_plus_t Pass::getCylinderAffordancePlus(std::string filename, Eigen::Isometry3d utorso_to_aff, int uid){ 
  Eigen::Isometry3d world_to_aff = world_to_utorso_*utorso_to_aff; 
  std::vector<double> xyzrpy = {.0 , .0 , .0, 0. , 0 , 0};
  xyzrpy[0] = world_to_aff.translation().x();
  xyzrpy[1] = world_to_aff.translation().y();
  xyzrpy[2] = world_to_aff.translation().z();
  quat_to_euler ( Eigen::Quaterniond(world_to_aff.rotation() ), xyzrpy[3], xyzrpy[4], xyzrpy[5] );  
  
  drc::affordance_plus_t p;
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="cylinder";
  a.aff_store_control = drc::affordance_t::NEW;
  
  a.params.push_back(0.5  ); a.param_names.push_back("length");
  a.params.push_back(0.03  ); a.param_names.push_back("radius");
  a.params.push_back(1.0  ); a.param_names.push_back("mass");
  a.nparams =a.params.size();
  a.nstates =0;

  a.origin_xyz[0]=xyzrpy[0]; a.origin_xyz[1]=xyzrpy[1]; a.origin_xyz[2]=xyzrpy[2]; 
  a.origin_rpy[0]=xyzrpy[3]; a.origin_rpy[1]=xyzrpy[4]; a.origin_rpy[2]=xyzrpy[5]; 
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0;   
 
  p.aff = a;
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  p.npoints=0; 
  p.ntriangles =0;
  
  p.aff.bounding_lwh[0]=0.36;       p.aff.bounding_lwh[1]=0.33;      p.aff.bounding_lwh[2]=0.3;   
  return p;
}


drc::affordance_plus_t Pass::getBoxAffordancePlus(std::string filename, Eigen::Isometry3d utorso_to_aff, int uid){ 
  Eigen::Isometry3d world_to_aff = world_to_utorso_*utorso_to_aff; 
  std::vector<double> xyzrpy = {.0 , .0 , .0, 0. , 0 , 0};
  xyzrpy[0] = world_to_aff.translation().x();
  xyzrpy[1] = world_to_aff.translation().y();
  xyzrpy[2] = world_to_aff.translation().z();
  quat_to_euler ( Eigen::Quaterniond(world_to_aff.rotation() ), xyzrpy[3], xyzrpy[4], xyzrpy[5] );  
  
  drc::affordance_plus_t p;
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="box";
  a.aff_store_control = drc::affordance_t::NEW;
  
  a.params.push_back(0.10  ); a.param_names.push_back("lX");
  a.params.push_back(0.05  ); a.param_names.push_back("lY");
  a.params.push_back(1.5  ); a.param_names.push_back("lZ");
  a.nparams =a.params.size();
  a.nstates =0;

  a.origin_xyz[0]=xyzrpy[0]; a.origin_xyz[1]=xyzrpy[1]; a.origin_xyz[2]=xyzrpy[2]; 
  a.origin_rpy[0]=xyzrpy[3]; a.origin_rpy[1]=xyzrpy[4]; a.origin_rpy[2]=xyzrpy[5]; 
  a.bounding_xyz[0]=0.0; a.bounding_xyz[1]=0; a.bounding_xyz[2]=0; 
  a.bounding_rpy[0]=0.0; a.bounding_rpy[1]=0.0; a.bounding_rpy[2]=0.0;   
 
  p.aff = a;
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  p.npoints=0; 
  p.ntriangles =0;
  
  p.aff.bounding_lwh[0]=0.1;       p.aff.bounding_lwh[1]=0.05;      p.aff.bounding_lwh[2]=1.5;   
  return p;
}



void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  
  frames_cpp_->get_trans_with_utime( "utorso" , "local", msg->utime, world_to_utorso_);
  
  /*
  // 0. Extract World Pose of body:
  world_to_utorso_.setIdentity();
  world_to_utorso_.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_utorso_.rotate(quat);    
  */
  
  drc::affordance_plus_t aff;
  
  if (which_affordance_ ==0){
    Eigen::Isometry3d utorso_to_aff(Eigen::Isometry3d::Identity());
    utorso_to_aff.translation()  << 0.45, 0.1, 0.3;
    utorso_to_aff.rotate( Eigen::Quaterniond(euler_to_quat(90*M_PI/180,0*M_PI/180,70*M_PI/180)) );    
    aff = getSteeringCylinderAffordancePlus("notused", utorso_to_aff, 1);
  }else if (which_affordance_ ==1){
    Eigen::Isometry3d utorso_to_aff(Eigen::Isometry3d::Identity());
    utorso_to_aff.translation()  << 0.45, -0.1, 0.3;
    utorso_to_aff.rotate( Eigen::Quaterniond(euler_to_quat(90*M_PI/180,0*M_PI/180,-70*M_PI/180)) );    
    aff = getSteeringCylinderAffordancePlus("notused", utorso_to_aff, 1);
  }else if (which_affordance_ ==2){
    Eigen::Isometry3d utorso_to_aff(Eigen::Isometry3d::Identity());
    utorso_to_aff.translation()  << 0.55, 0.0, 0.0;
    utorso_to_aff.rotate( Eigen::Quaterniond(euler_to_quat(90*M_PI/180,0*M_PI/180,0*M_PI/180)) );    
    aff = getBoxAffordancePlus("notused", utorso_to_aff, 1);
  }else if (which_affordance_ ==3){
    Eigen::Isometry3d utorso_to_aff(Eigen::Isometry3d::Identity());
    utorso_to_aff.translation()  << 0.55, 0.0, 0.0;
    utorso_to_aff.rotate( Eigen::Quaterniond(euler_to_quat(0*M_PI/180,0*M_PI/180,0*M_PI/180)) );
    aff = getCylinderAffordancePlus("notused", utorso_to_aff, 1);
  }else{
    std::cout << "Affordance not recognised\n";
    exit(-1); 
  }
  
  lcm_->publish("AFFORDANCE_FIT",&aff);
  
  exit(-1);  
}



int main(int argc, char ** argv) {
  std::cout << "0 - Steering Cylinder at left shoulder\n";
  std::cout << "1 - Steering Cylinder at right shoulder\n";
  std::cout << "2 - 2x4 low\n";
  std::cout << "3 - cylinder pipe\n";
  
  
  string mode = "both";
  int which_affordance = 0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(mode, "m", "mode","mode: both, grasp, plan");
  opt.add(which_affordance, "a", "affordance","Which Affordance to initialise");
  opt.parse();
  std::cout << "mode: " << mode << "\n";  

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,mode,which_affordance);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}