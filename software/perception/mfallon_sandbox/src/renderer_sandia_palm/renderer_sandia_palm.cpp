#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>

#include "renderer_sandia_palm.hpp"
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

  std::string mesh_filename = "/home/mfallon/drc/software/models/mit_gazebo_models/sandia_hand/meshes/palm.ply";
  std::cout << "About to read: " << mesh_filename << std::endl;
  pcl::PolygonMesh combined_mesh;	// (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile (mesh_filename, combined_mesh);
  pcl::PolygonMesh::Ptr combined_mesh_ptr_temp (new pcl::PolygonMesh (combined_mesh));
  combined_mesh_ptr_ = combined_mesh_ptr_temp;

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
  pc_vis_->obj_cfg_list.push_back( obj_cfg(6001,"Right Palm",5,1) );
  lcm_->subscribe("EST_ROBOT_STATE",&joints2frames::robot_state_handler,this);  

  float colors_b[] ={0.0,0.0,0.0};
  std::vector<float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(6002,"Mesh"     ,7,1, 6001,0, colors_v ));


  pc_vis_->mesh_to_lcm_from_list(6002, combined_mesh_ptr_ , 0 ,0);
  
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


void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  
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
    if(  (*ii).first.compare( "right_palm" ) == 0 ){
      std::cout << "i got right palm\n";

      int counter =msg->utime;  
      std::vector<Isometry3dTime> world_to_jointsT;
      Eigen::Isometry3d body_to_joint = KDLToEigen( (*ii).second );
      // convert to world positions
      Isometry3dTime world_to_jointT(0, world_to_body*body_to_joint);
      world_to_jointsT.push_back(world_to_jointT);
      pc_vis_->pose_collection_to_lcm_from_list(6001, world_to_jointsT); // all joints in world frame


    }
  }

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
