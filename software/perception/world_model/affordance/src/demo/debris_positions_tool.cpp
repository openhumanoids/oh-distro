// given these standing position offsets. need to pair aff with positions
// 
// next:
// given an aff. find near corner point, publish walk goal relative to there

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
#include <boost/lexical_cast.hpp>

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
using namespace boost::assign; // bring 'operator+()' into scope
using namespace boost;


using namespace std;
char* pDRC_BASE = getenv("DRC_BASE");  
string drc_base = string(pDRC_BASE);

// Raw details read from affordance text file
class AffRaw { 
public:
  AffRaw(std::string friendly_name_, Eigen::Isometry3d standing_position_):
    friendly_name_ (friendly_name_), standing_position_(standing_position_){};
  string friendly_name_;
  
  Eigen::Isometry3d standing_position_;
  
};


struct Config{
  
  int server_id;
  int library_id;
  
  Config () {
        server_id = 1;
        library_id = 1;
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
    bool cartpos_ready_, leftfoot_ready_, walkinggoal_ready_;
    int64_t current_utime_;
    
    void affHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::affordance_plus_collection_t* msg);     
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);
    void poseGroundHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    void poseLeftFootHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    void poseRightFootHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;
    
    
    void getCurrentStandingPositionAsRelative(Eigen::Vector3d min_pt);
    void getPriorStandingPositionAsRelative(Eigen::Vector3d min_pt, Eigen::Isometry3d corner_to_walking_goa);
    
    
    void readStandingPositionsFile(std::string filename, std::vector<AffRaw> &affraw_list);
    
    // Affordances stored using the object_name that Sisir seems to be sending with INIT_GRASP_OPT_* messages
    // [otdf_type]_[uid]
    Eigen::Isometry3d world_to_body_high_;
    Eigen::Isometry3d world_to_leftfoot_,world_to_rightfoot_, world_to_standing_;
    drc::affordance_t aff_;
    map<string, drc::affordance_t > affs_;
    
    double ground_height_;
    
    AffordanceUtils affutils_;
    
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, Config& config_):
    lcm_(lcm_),config_(config_){
  cartpos_ready_ = false;
  leftfoot_ready_ = false;
  walkinggoal_ready_ = false;
      
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = new KDL::TreeFkSolverPosFull_recursive(tree);
      
  lcm_->subscribe( "AFFORDANCE_PLUS_COLLECTION" ,&Pass::affHandler,this);
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robot_state_handler,this);  
  lcm_->subscribe( "POSE_GROUND" ,&Pass::poseGroundHandler,this);
  lcm_->subscribe( "POSE_LEFT_FOOT" ,&Pass::poseLeftFootHandler,this);  
  lcm_->subscribe( "POSE_RIGHT_FOOT" ,&Pass::poseRightFootHandler,this);  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(600000,"Near Corner",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(600001,"Current walking goal",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(600002,"Robot to Corner",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(600003,"Next Walking Goal",5,1) );

  
}

void Pass::poseGroundHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  ground_height_ = msg->pos[2];
}

void Pass::poseLeftFootHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  world_to_leftfoot_.setIdentity();
  world_to_leftfoot_.translation() = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]) ;
  Eigen::Quaterniond quat = Eigen::Quaterniond( msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3] );
  world_to_leftfoot_.rotate(quat);      
  leftfoot_ready_=true;
}

void Pass::poseRightFootHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  if (!leftfoot_ready_){
    return;
  }
  
  world_to_rightfoot_.setIdentity();
  world_to_rightfoot_.translation() = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]) ;
  Eigen::Quaterniond quat = Eigen::Quaterniond( msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3] );
  world_to_rightfoot_.rotate(quat);      
  
  // strip out the yaw only:
  double rpy[3];
  quat_to_euler ( quat , rpy[0], rpy[1], rpy[2] );
  quat=  euler_to_quat( 0,0,rpy[2]  ); 
  
  world_to_standing_.setIdentity();
  world_to_standing_.translation() = ( world_to_rightfoot_.translation()  + world_to_leftfoot_.translation() )/2;  // mid foot position
  world_to_standing_.rotate ( quat);
  
  walkinggoal_ready_ = true;
}

void Pass::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  
  
  // 0. Extract World Pose of body:
  world_to_body_high_.setIdentity();
  // edit: use a point thats very high to force the choice of high points
  world_to_body_high_.translation()  << msg->pose.translation.x, msg->pose.translation.y, 1000 ;//, msg->pose.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  world_to_body_high_.rotate(quat);    
  
  /*
    
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
  */
  cartpos_ready_=true;
  
}


// given a point under the nearmost corner and a look direciton
// what is the current relative standing position?
void Pass::getCurrentStandingPositionAsRelative(Eigen::Vector3d min_pt){
  // A point on the ground under the nearest corner facing along the plank
  Eigen::Isometry3d world_to_corner(Eigen::Isometry3d::Identity());
  world_to_corner.translation() << min_pt(0), min_pt(1), 0 ; // point on the ground
  world_to_corner.rotate( euler_to_quat(0,0, aff_.origin_rpy[2]) );
  Isometry3dTime pt_poseT = Isometry3dTime(current_utime_, world_to_corner );
  pc_vis_->pose_to_lcm_from_list(600000, pt_poseT);   
  
  Isometry3dTime current_walking_goalT = Isometry3dTime(current_utime_,  world_to_standing_  );
  pc_vis_->pose_to_lcm_from_list(600001, current_walking_goalT);   
  
  
  Eigen::Isometry3d robot_to_corner =  world_to_corner.inverse() * world_to_standing_ ;
  
  Isometry3dTime robot_to_cornerT = Isometry3dTime(current_utime_,  robot_to_corner  );
  pc_vis_->pose_to_lcm_from_list(600002, robot_to_cornerT);   
  
  Eigen::Quaterniond quat(robot_to_corner.rotation());
  double rpy[3];
  quat_to_euler(quat, rpy[0],rpy[1],rpy[2]);  
  
  cout << robot_to_corner.translation().transpose() << " is pose\n";
  cout << rpy[2] << " is yaw [" << rpy[2]*180/M_PI <<"]\n";

  cout << robot_to_corner.translation().transpose().x() << ", " 
       << robot_to_corner.translation().transpose().y() << ", "
       << rpy[2] << "\n";

  exit(-1);
}


void Pass::getPriorStandingPositionAsRelative(Eigen::Vector3d min_pt, Eigen::Isometry3d corner_to_walking_goal){
  // A point on the ground under the nearest corner facing along the plank
  Eigen::Isometry3d world_to_corner(Eigen::Isometry3d::Identity());
  world_to_corner.translation() << min_pt(0), min_pt(1), 0 ; // point on the ground
  world_to_corner.rotate( euler_to_quat(0,0, aff_.origin_rpy[2]) );
  Isometry3dTime pt_poseT = Isometry3dTime(current_utime_, world_to_corner );
  pc_vis_->pose_to_lcm_from_list(600000, pt_poseT);   
  
  
  Eigen::Isometry3d world_to_walking_goal =  world_to_corner * corner_to_walking_goal;
  
  Isometry3dTime world_to_walking_goalT = Isometry3dTime(current_utime_,  world_to_walking_goal  );
  pc_vis_->pose_to_lcm_from_list(600003, world_to_walking_goalT);   

  
  exit(-1);
}





void Pass::readStandingPositionsFile(std::string filename, std::vector<AffRaw> &affraw_list){

  AffRaw affrawA("null", Eigen::Isometry3d::Identity()) ;
  affraw_list.push_back(affrawA);       
  
  ifstream fileinput (filename);
  if (fileinput.is_open()){
    string message;
    while ( getline (fileinput,message) ) { // for each line
      std::string letter1 = message.substr (0,1);  
      if (letter1 == "#"){
        continue;
      }
      //cout << message << endl;
      

      vector<string> tokens;
      boost::split(tokens, message, boost::is_any_of(","));
      vector<double> values;
      for (size_t i=1 ; i < 5 ; i++){
        //std::cout << tokens[i] << "\n";
        values.push_back(lexical_cast<double>( tokens[i] ));
      }
      
      
      Eigen::Isometry3d standing_position = Eigen::Isometry3d::Identity();
      standing_position.translation() << values[0], values[1], 0;
      // put the feet on the ground
      // values[2];
      Eigen::Quaterniond quat = euler_to_quat( 0,0, values[3]);
      standing_position.rotate(quat);
      
      AffRaw affraw = AffRaw(tokens[0], standing_position) ;
      affraw_list.push_back(affraw);       
      
      
    }
  }
  fileinput.close();

       
    
}




void Pass::affHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  if (!cartpos_ready_){
    return;
  }
  if (!walkinggoal_ready_){
    return;
  }
  current_utime_ = msg->utime;
  std::cout << "got "<< msg->naffs << " affs\n";
  
  for (int i=0 ; i < msg->naffs ; i++){
    drc::affordance_t aff = msg->affs_plus[i].aff;
    std::stringstream ss;
    ss << aff.otdf_type << '_' << aff.uid;    
    // std::cout << ss.str() << "\n";
    affs_[ ss.str() ]=  aff;
    
  }     
  
  std::stringstream object_name_ss;
  object_name_ss << "box_" << config_.server_id;
  
  
  std::string object_name = object_name_ss.str();//"box_9";
  map< string , drc::affordance_t >::iterator it = affs_.find(object_name  );
  if (it == affs_.end() ){
    std::cout << "couldn't find affordance of name ["<< object_name << "]\n";
    return;
  }else{
    aff_ = it->second;
  }

  std::cout << "Found to " <<  object_name << "\n";

  // Extract affordance info:
  std::map<string,double> am;
  for (int j=0; j< aff_.nparams; j++){
    am[ aff_.param_names[j] ] = aff_.params[j];
  }
  Eigen::Vector3d aff_len( am.find("lX")->second, am.find("lY")->second, am.find("lZ")->second );
  Eigen::Affine3d world_to_aff_affine = Eigen::Affine3d( affutils_.getPose(aff_.origin_xyz, aff_.origin_rpy) );
  
  // Find the point nearest to the robot:
  double min_dist =99999.9;
  Eigen::Vector3d min_pt(99999.9, 99999.9, 99999.9);
  for (int ix=-1; ix<=1; ix=ix+2){
    for (int iy=-1; iy<=1; iy=iy+2){
      for (int iz=-1; iz<=1; iz=iz+2){      
        //std::cout << ix << " " << iy << " " << iz << "\n";
        
        Eigen::Vector3d pt = Eigen::Vector3d(ix*aff_len(0)/2, iy*aff_len(1)/2, iz*aff_len(2)/2 );
        pt = world_to_aff_affine * pt;  
        double distance = (world_to_body_high_.translation() - pt) .norm();
        //cout << world_to_body_high_.translation().transpose() << " w\n";
        //cout << pt.transpose() << " p\n";
        //cout << distance << " distance\n";
        if(distance < min_dist){
          min_dist = distance;
          min_pt = pt;
        }
        
      }
    }
  }
  //cout << min_dist << " min dist\n";
  //cout << min_pt.transpose() << " the p\n";
  
  
  if ( config_.library_id  == -1){
    getCurrentStandingPositionAsRelative(min_pt);
  }else{
    string standing_filename_full = string(drc_base + "/software/config/task_config/debris/debrisStandXYZYaw.csv");
    std::vector<AffRaw> affraw_list;// = readAffordanceFile(debris_filename_full);
    std:: cout << affraw_list.size() << " affordances read\n";

    readStandingPositionsFile( standing_filename_full , affraw_list);
    getPriorStandingPositionAsRelative(min_pt, affraw_list[config_.library_id].standing_position_ );
    
    
  }

    

  
  
}


int main(int argc, char ** argv) {
  Config config;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(config.server_id, "s", "server_id","Aff Server Id");
  opt.add(config.library_id, "l", "library_id","Lib Id");
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
