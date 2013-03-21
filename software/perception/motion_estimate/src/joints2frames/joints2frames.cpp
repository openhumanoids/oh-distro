// LCM example program for interacting  with PCL data
// In this case it pushes the data back out to LCM in
// a different message type for which the "collections"
// renderer can view it in the LCM viewer
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

#include "joints2frames.hpp"
#include <ConciseArgs>

using namespace std;
using namespace boost;
using namespace boost::assign;

#define DO_TIMING_PROFILE FALSE


/////////////////////////////////////

joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm, bool show_labels_, bool show_triads_):
          lcm_(publish_lcm), _urdf_parsed(false), show_labels_(show_labels_), show_triads_(show_triads_),
          world_to_bodyT_(0, Eigen::Isometry3d::Identity()),
          body_to_headT_(0, Eigen::Isometry3d::Identity()) {

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(6001,"Frames",5,1) );

  lcm_->subscribe("EST_ROBOT_STATE",&joints2frames::robot_state_handler,this);  
  _urdf_subscription =   lcm_->subscribe("ROBOT_MODEL",&joints2frames::urdf_handler,this);  
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (!_urdf_parsed){
    cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
    return;
  }

  
  #if DO_TIMING_PROFILE
    std::vector<int64_t> tic_toc;
    tic_toc.push_back(_timestamp_now());
  #endif
  
  
  // 1. Solve for Forward Kinematics:
  _link_tfs.clear();
  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  map<string, drc::transform_t > cartpos_out;
  for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));

  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
  
  
  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
  kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
  
  
  // 2a. Determine the required BOT_FRAMES transforms:
  Eigen::Isometry3d body_to_head, body_to_hokuyo_link;
  bool body_to_head_found =false;
  bool body_to_hokuyo_link_found = false;
  for( map<string, drc::transform_t>::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string joint = (*ii).first;
    if (   (*ii).first.compare( "head" ) == 0 ){
      body_to_head.setIdentity();
      body_to_head.translation()  << (*ii).second.translation.x, (*ii).second.translation.y, (*ii).second.translation.z;
      Eigen::Quaterniond quat = Eigen::Quaterniond((*ii).second.rotation.w, (*ii).second.rotation.x, (*ii).second.rotation.y, (*ii).second.rotation.z);
      body_to_head.rotate(quat);    
      body_to_head_found=true;
    }else if(  (*ii).first.compare( "hokuyo_link" ) == 0 ){
      body_to_hokuyo_link.setIdentity();
      body_to_hokuyo_link.translation()  << (*ii).second.translation.x, (*ii).second.translation.y, (*ii).second.translation.z;
      Eigen::Quaterniond quat = Eigen::Quaterniond((*ii).second.rotation.w, (*ii).second.rotation.x, (*ii).second.rotation.y, (*ii).second.rotation.z);
      body_to_hokuyo_link.rotate(quat);    
      body_to_hokuyo_link_found=true;
    }
  }  
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
  
  
  // 2b. Republish the required BOT_FRAMES transforms:
  if (body_to_head_found){
    bot_core::rigid_transform_t tf;
    tf.utime = msg->utime;
    tf.trans[0] = body_to_head.translation().x();
    tf.trans[1] = body_to_head.translation().y();
    tf.trans[2] = body_to_head.translation().z();
    Eigen::Quaterniond quat = Eigen::Quaterniond( body_to_head.rotation() );
    tf.quat[0] = quat.w();
    tf.quat[1] = quat.x();
    tf.quat[2] = quat.y();
    tf.quat[3] = quat.z();
    lcm_->publish("BODY_TO_HEAD", &tf);     
    
    if (body_to_hokuyo_link_found){
      Eigen::Isometry3d head_to_hokuyo_link = body_to_head.inverse() * body_to_hokuyo_link ;
      
      bot_core::rigid_transform_t tf;
      tf.utime = msg->utime;
      tf.trans[0] = head_to_hokuyo_link.translation().x();
      tf.trans[1] = head_to_hokuyo_link.translation().y();
      tf.trans[2] = head_to_hokuyo_link.translation().z();
      Eigen::Quaterniond quat = Eigen::Quaterniond( head_to_hokuyo_link.rotation() );
      tf.quat[0] = quat.w();
      tf.quat[1] = quat.x();
      tf.quat[2] = quat.y();
      tf.quat[3] = quat.z();
      lcm_->publish("HEAD_TO_HOKUYO_LINK", &tf);     
    }
  }
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"createScene");  
  #endif

    
  if (!show_triads_){
    return; 
  }
  
  // 3. Extract World Pose:
  world_to_bodyT_.pose.setIdentity();
  world_to_bodyT_.pose.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  world_to_bodyT_.pose.rotate(quat);    
  world_to_bodyT_.utime = msg->utime;
  
  // 4. Loop through joints and extract world positions:
  int counter =msg->utime;  
  std::vector<Isometry3dTime> body_to_jointTs, world_to_jointsT;
  std::vector< int64_t > body_to_joint_utimes;
  std::vector< std::string > joint_names;
  for( map<string, drc::transform_t>::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string joint = (*ii).first;
    //cout << joint  << ": \n";
    joint_names.push_back( joint  );
    body_to_joint_utimes.push_back( counter);
    
    Eigen::Isometry3d body_to_joint;
    body_to_joint.setIdentity();
    body_to_joint.translation()  << (*ii).second.translation.x, (*ii).second.translation.y, (*ii).second.translation.z;
    Eigen::Quaterniond quat = Eigen::Quaterniond((*ii).second.rotation.w, (*ii).second.rotation.x, (*ii).second.rotation.y, (*ii).second.rotation.z);
    body_to_joint.rotate(quat);    
    Isometry3dTime body_to_jointT(counter, body_to_joint);
    body_to_jointTs.push_back(body_to_jointT);
    // convert to world positions
    Isometry3dTime world_to_jointT(counter, world_to_bodyT_.pose*body_to_joint);
    world_to_jointsT.push_back(world_to_jointT);
    counter++;
  }
  
  pc_vis_->pose_collection_to_lcm_from_list(6001, world_to_jointsT); // all joints in world frame
  if (show_labels_)
    pc_vis_->text_collection_to_lcm(6002, 6001, "Frames [Labels]", joint_names, body_to_joint_utimes );    
}


void joints2frames::urdf_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
    const  drc::robot_urdf_t* msg)
{
  // Received robot urdf string. Store it internally and get all available joints.
  _robot_name      = msg->robot_name;
  _urdf_xml_string = msg->urdf_xml_string;
  cout<< "Received urdf_xml_string of robot [" 
      << msg->robot_name << "], storing it internally as a param" << endl;

  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model robot_model; 
  if (!robot_model.initString( msg->urdf_xml_string))
  {
    cerr << "ERROR: Could not generate robot model" << endl;
  }

  typedef map<string, shared_ptr<urdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
  {
    if(it->second->type!=6) // All joints that not of the type FIXED.
      _joint_names_.push_back(it->first);
  }

  _links_map =  robot_model.links_;

  //---------parse the tree and stop listening for urdf messages

  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  {
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    return;
  }

  //unsubscribe from urdf messages
  lcm_->unsubscribe(_urdf_subscription); 

  //
  _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  //remember that we've parsed the urdf already
  _urdf_parsed = true;

  cout<< "Number of Joints: " << _joint_names_.size() <<endl;
};// end handleRobotUrdfMsg


int
main(int argc, char ** argv){
  string role = "robot";
  bool labels = false;
  bool triads = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(role, "r", "role","Role - robot or base");
  opt.add(triads, "t", "triads","Frame Triads - show no not");
  opt.add(labels, "l", "labels","Frame Labels - show no not");
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
  
  joints2frames app(lcm,labels,triads);
  while(0 == lcm->handle());
  return 0;
}

