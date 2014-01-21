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
#include "lcmtypes/drc/affordance_plus_t.hpp"
#include "lcmtypes/drc/affordance_plus_collection_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/robot_plan_w_keyframes_t.hpp"
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
  int aff_id_;
  
  Config () {
        use_sandia_ = false;
        use_left_hand_ = false;
        aff_id_ = -1;
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
    boost::shared_ptr<rgbd_primitives>  prim_; // this should be moved into the library
    bool cartpos_ready_;
    
    void affHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::affordance_plus_collection_t* msg);      
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);

    void manipPlanHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg);    

    
    boost::shared_ptr<ModelClient> model_;
    KDL::TreeFkSolverPosFull_recursive* fksolver_;
    map<string, KDL::Frame > cartpos_;

    drc::robot_state_t rstate_;

    bool rstate_init_;
    bool   aff_init_;

    
    std::vector <drc::affordance_plus_t> affs_;
    
    drc::affordance_plus_t aff_;
    Eigen::Isometry3d world_to_body_;
    
    AffordanceUtils affutils_;
    
    void solveFK(drc::robot_state_t state, Eigen::Isometry3d &world_to_body, 
                 map<string, KDL::Frame > &cartpos, bool &cartpos_ready  );    
    std::string getPalmLink();
    int getHandType();
    
    Eigen::Isometry3d palm_to_hose_;
    
    void affordancePlusInterpret(drc::affordance_plus_t affplus, float aff_uid, pcl::PolygonMesh::Ptr &mesh_out);
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, Config& config_):
    lcm_(lcm_),config_(config_){
      
  rstate_init_ = false;
  cartpos_ready_ = false;
  aff_init_= false;

      
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = new KDL::TreeFkSolverPosFull_recursive(tree);
      
  lcm_->subscribe("EST_ROBOT_STATE",&Pass::robot_state_handler,this);  
  lcm_->subscribe("CANDIDATE_MANIP_PLAN",&Pass::manipPlanHandler,this);
  
  lcm_->subscribe( "AFFORDANCE_PLUS_COLLECTION" ,&Pass::affHandler,this);

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9994,"Pose - Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(9993,"Mesh - Object"     ,7,1, 9994,0, { 0.0, 1.0, 0.0} ));
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
}



void Pass::manipPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_w_keyframes_t* msg){
  //std::cout << "got "<< msg->naffs << " affs\n";
  if(rstate_init_){
      solveFK(rstate_,world_to_body_, cartpos_, cartpos_ready_);
  }
  if (!cartpos_ready_){
   std::cout << "no rstate yet\n";
   return; 
  }
  if (!aff_init_){
   std::cout << "no affs yet\n";
   return; 
  }
  

  // Make a poly mesh of the hypothesied positions of the mesh:
  pcl::PolygonMesh::Ptr full_mesh(new pcl::PolygonMesh());
  
  if (affs_.size() ==0){
    std::cout << "no affs present\n";
    return;
  }
  
  std::cout << "creating mesh with " << affs_.size() << " affs\n";
  
  for (size_t  j=0; j< affs_.size() ; j++){
    drc::affordance_plus_t this_aff  = affs_[j];
  
    // Determine the current hand to aff pose: 
    Eigen::Isometry3d body_to_palm = KDLToEigen(cartpos_.find( getPalmLink() )->second);
    Eigen::Isometry3d world_to_palm =  world_to_body_* body_to_palm;

    Eigen::Isometry3d world_to_hose = affutils_.getPose(this_aff.aff.origin_xyz, this_aff.aff.origin_rpy );
    palm_to_hose_ = world_to_palm.inverse() * world_to_hose;

  
    for (size_t i=0; i < msg->plan.size() ; i++){     
      map<string, KDL::Frame > cartpos_plan;
      bool cartpos_ready_plan = false;
      Eigen::Isometry3d world_to_body_plan;
      solveFK(msg->plan[ i],world_to_body_plan, cartpos_plan, cartpos_ready_plan);

      Eigen::Isometry3d body_to_palm = KDLToEigen(cartpos_plan.find( getPalmLink() )->second);
      Eigen::Isometry3d world_to_palm =  world_to_body_plan* body_to_palm;
      Eigen::Isometry3d world_to_hose =  world_to_palm* palm_to_hose_;
      affutils_.setXYZRPYFromIsometry3d(this_aff.aff.origin_xyz, this_aff.aff.origin_rpy, world_to_hose);

      std::stringstream ss ;
      print_Isometry3d(palm_to_hose_, ss);
      std::cout << ss.str() << " [palm to hose]\n";

      {
      std::stringstream ss ;
      print_Isometry3d(palm_to_hose_, ss);
      std::cout << ss.str() << " [palm to hose]\n";
      }
      
      pcl::PolygonMesh::Ptr new_aff_mesh(new pcl::PolygonMesh());
      float jet_color = ((float) i) / (msg->plan.size() -1);
      affordancePlusInterpret(this_aff, jet_color, new_aff_mesh );
      pc_vis_->mergePolygonMesh(full_mesh,new_aff_mesh  );
    }
  
  }
  
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(0, null_pose);  
  pc_vis_->pose_to_lcm_from_list(9994, null_poseT);
  pc_vis_->mesh_to_lcm_from_list(9993, full_mesh , 0 , 0);
  
}



pcl::PolygonMesh::Ptr getPolygonMesh(std::string filename){
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(  filename    ,mesh);
  pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh(mesh));
  cout << "read in :" << filename << "\n"; 
  //state->model = mesh_ptr;  
  return mesh_ptr;
}


void setPolygonMeshColor( pcl::PolygonMesh::Ptr &mesh, int r,int g, int b ){
  pcl::PointCloud<pcl::PointXYZRGB> mesh_cloud_1st;  
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ;  
  pcl::fromROSMsg(mesh->cloud, cloudXYZ);

  // this was pcl::PointXYZRGB omly until recently
  for (size_t i=0;i< cloudXYZ.points.size() ; i++){
    pcl::PointXYZRGB pt;
    pt.x = cloudXYZ.points[i].x;
    pt.y = cloudXYZ.points[i].y;
    pt.z = cloudXYZ.points[i].z;
    pt.r = r;
    pt.g = g;
    pt.b = b;
    mesh_cloud_1st.points.push_back(pt);
  }
      
  // transform
  pcl::toROSMsg (mesh_cloud_1st, mesh->cloud);      
}




void Pass::affordancePlusInterpret(drc::affordance_plus_t affplus, float aff_uid, pcl::PolygonMesh::Ptr &mesh_out){ 
    std::map<string,double> am;
    for (size_t j=0; j< affplus.aff.nparams; j++){
      am[ affplus.aff.param_names[j] ] = affplus.aff.params[j];
    }

    //Eigen::Isometry3d transform = affutils.getPose(affplus.aff.param_names, affplus.aff.params);
    Eigen::Isometry3d transform = affutils_.getPose(affplus.aff.origin_xyz, affplus.aff.origin_rpy );
    
    string otdf_type = affplus.aff.otdf_type;
    
    if (otdf_type == "box"){
      //cout  << aff_uid << " is a box\n";
      mesh_out = prim_->getCubeWithTransform(transform,am.find("lX")->second, am.find("lY")->second, am.find("lZ")->second);
    }else if(otdf_type == "cylinder"){
      //cout  << aff_uid << " is a cylinder\n";
      mesh_out = prim_->getCylinderWithTransform(transform, am.find("radius")->second, am.find("radius")->second, am.find("length")->second );
    }else if(otdf_type == "steering_cyl"){
      //cout  << aff_uid << " is a steering_cyl\n";
      mesh_out = prim_->getCylinderWithTransform(transform, am.find("radius")->second, am.find("radius")->second, am.find("length")->second );
    }else if(otdf_type == "dynamic_mesh"){
      //cout  << aff_uid << " is a dynamic_mesh ["<< affplus.points.size() << " pts and " << affplus.triangles.size() << " tri]\n";
      mesh_out = affutils_.getMeshFromAffordance(affplus.points, affplus.triangles,transform);
    }else if(otdf_type == "dynamic_mesh_w_1_cylinder"){ // Ignores the cylinder and just draws the mesh
      //cout  << aff_uid << " is a dynamic_mesh_w_1_cylinder ["<< affplus.points.size() << " pts and " << affplus.triangles.size() << " tri]\n";
      mesh_out = affutils_.getMeshFromAffordance(affplus.points, affplus.triangles,transform);
    }else if(otdf_type == "plane"){
      //cout  << aff_uid << " is a plane ["<< affplus.points.size() << " pts and " << affplus.triangles.size() << " tri]\n";
      mesh_out = affutils_.getMeshFromAffordance(affplus.points, affplus.triangles,transform);
    }else if(otdf_type == "firehose"){ 
      // the simple two cylinder model maurice used in dec 2013
      // NB: I don't support otdf - so this is hard coded here for now
      // cout  << aff_uid << " is a firehose\n";
      mesh_out = prim_->getCylinderWithTransform(transform, 0.0266, 0.0266, 0.042 );
      Eigen::Isometry3d trans_2nd = Eigen::Isometry3d::Identity();
      trans_2nd.translation()  << 0,0, 0.033;      
      trans_2nd = transform * trans_2nd;
      pc_vis_->mergePolygonMesh(mesh_out, prim_->getCylinderWithTransform(trans_2nd, 0.031, 0.031, 0.024 ) );
    }else if(otdf_type == "wye_mesh"){ 
      //cout  << aff_uid << " is a wye_mesh\n";
      std::string fname = string(getenv( "DRC_BASE" )) + string( "/software/models/mit_gazebo_models/otdf/wye.obj");
      mesh_out = getPolygonMesh(fname);
      
      // Apply transform to polymesh:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::fromROSMsg(mesh_out->cloud, *cloud);  
      Eigen::Isometry3f pose_f = transform.cast<float>();
      Eigen::Quaternionf quat_f(pose_f.rotation());
      pcl::transformPointCloud (*cloud, *cloud,
      pose_f.translation(), quat_f); // !! modifies cloud
      pcl::toROSMsg(*cloud, mesh_out->cloud);       
      
    }else{
      cout  << aff_uid << " is a not recognised ["<< otdf_type <<"] not supported yet\n";
    }
    
    // Apply Some Coloring:
    float c[3];
    jet_rgb( aff_uid,c);
    setPolygonMeshColor(mesh_out, (int) round(c[0]*255), (int) round(c[1]*255) , (int) round(c[2]*255) );
    std::cout << aff_uid << " rgb range [0-1]\n";
}    






void Pass::affHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::affordance_plus_collection_t* msg){

  affs_ = msg->affs_plus;
  aff_init_ =true;
  
  
/*  for (int i=0 ; i < msg->naffs ; i++){
    if (msg->affs_plus[i].aff.uid ==  config_.aff_id_){
      aff_ = msg->affs_plus[i];
      aff_init_= true;
    }
  }
  */
}



int main(int argc, char ** argv) {
  Config config;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(config.use_left_hand_, "l", "use_left_hand","use left hand [defualt is right]");
  opt.add(config.aff_id_, "a", "aff_id","use this aff_id");
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
