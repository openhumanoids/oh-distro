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

double x_offset = 0;
double y_offset = 0;
double z_offset = 0;
bool rhand = false;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string mode_);
    
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

    BotParam* botparam_;
    bot::frames* frames_cpp_;
    
    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;
    
    
    drc::affordance_t aff_;
    Eigen::Isometry3d world_to_aff_ ;
    Eigen::Isometry3d world_to_body_;
    AffordanceUtils affutils_;
    
    vector<string> l_joint_name_;
    vector<string> r_joint_name_;
    vector<double> l_joint_position_;    
    vector<double> r_joint_position_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string mode_):
    lcm_(lcm_), mode_(mode_){
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
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60011,"Grasp Seed",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60001,"Grasp Frames",5,1) );
      

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
  r_joint_position_ = {0,0.7,0.7,  0,0.7,0.7,
                       0,0.7,0.7,  -0.15,0.68,0.47};  
}


drc::affordance_plus_t getSteeringCylinderAffordancePlus(std::string filename, std::vector<double> &xyzrpy, int uid){ 
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
  
  return p;
}



void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  
  frames_cpp_->get_trans_with_utime( "utorso" , "local", msg->utime, world_to_body_);
  
  /*
  // 0. Extract World Pose of body:
  world_to_body_.setIdentity();
  world_to_body_.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body_.rotate(quat);    
  */
  
  double y_offcenter = 0.1;
  double rpy_sign = 1;
  if (rhand){
    y_offcenter = -1.0*y_offcenter;
    rpy_sign = -1;
  }
  
  Eigen::Isometry3d body_to_aff;
  body_to_aff.setIdentity();
  body_to_aff.translation()  << 0.50 +x_offset, y_offcenter +y_offset, 0.2 +z_offset; // away 
  Eigen::Quaterniond quat2 = Eigen::Quaterniond(  euler_to_quat( rpy_sign* 90*M_PI/180 ,rpy_sign*  0*M_PI/180 ,rpy_sign*-50*M_PI/180  )  );
  body_to_aff.rotate(quat2);    

  Eigen::Isometry3d world_to_aff = world_to_body_*body_to_aff; 
  
  
  drc::ee_goal_t msg_out;
  msg_out.utime = bot_timestamp_now();

  msg_out.ee_goal_pos.translation.x = world_to_aff.translation().x();
  msg_out.ee_goal_pos.translation.y = world_to_aff.translation().y();
  msg_out.ee_goal_pos.translation.z = world_to_aff.translation().z();
  Eigen::Quaterniond quat3 = Eigen::Quaterniond(world_to_aff.rotation() );
  msg_out.ee_goal_pos.rotation.w = quat3.w();
  msg_out.ee_goal_pos.rotation.x = quat3.x();
  msg_out.ee_goal_pos.rotation.y = quat3.y();
  msg_out.ee_goal_pos.rotation.z = quat3.z();
  msg_out.num_chain_joints = 0;
  
  if (rhand){
    lcm_->publish("LEFT_PALM_GOAL_CLEAR", &msg_out);  
    sleep(1);
    lcm_->publish("RIGHT_PALM_GOAL", &msg_out);  
    
  }else{
    lcm_->publish("RIGHT_PALM_GOAL_CLEAR", &msg_out);  
    sleep(1);
    lcm_->publish("LEFT_PALM_GOAL", &msg_out);  
  }

  exit(-1);
  
}



int main(int argc, char ** argv) {
  string mode = "both";
  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(mode, "m", "mode","mode: both, grasp, plan");
  opt.add(x_offset, "x", "x","x");
  opt.add(y_offset, "y", "y","y");
  opt.add(z_offset, "z", "z","z");
  opt.add(rhand, "r", "rhand","Use Right Hand (otherwise left)");
  opt.parse();
  std::cout << "x: " << x_offset << "\n";  
  std::cout << "y: " << y_offset << "\n";  
  std::cout << "z: " << z_offset << "\n";  
  std::cout << "rhand: " << rhand       << "\n";  

  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,mode);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}