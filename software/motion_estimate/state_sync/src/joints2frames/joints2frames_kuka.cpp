#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>

#include "joints2frames_kuka.hpp"
#include <ConciseArgs>

using namespace std;
using namespace boost;
using namespace boost::assign;

// false usually, set true to disable the limiting:
#define DONT_LIMIT_FREQUENCY FALSE

joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &lcm_, bool show_labels_, bool show_triads_):
          lcm_(lcm_), show_labels_(show_labels_), show_triads_(show_triads_){
            
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
            
            
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString( model_->getURDFString() ,tree)){
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl;
    exit(-1);
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  // Vis Config:
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(6001,"Frames",5,1) );
  lcm_->subscribe("EST_ROBOT_STATE",&joints2frames::robot_state_handler,this);  

  #if DONT_LIMIT_FREQUENCY
    std::cout << "Output signals will not limited in rate\n";  
  #else
    std::cout << "Output signals will be limited to these rates:\n";
    pub_frequency_["BODY_TO_LWR_ARM_7_LINK"] = FrequencyLimit(0, 1E6/100 );
  #endif

}


double joints2frames::getMaxFrequency(std::string query_root){
  double max_frequency=1.0;  
  
  string query = "coordinate_frames." + query_root+ ".max_frequency";
  
  if ( bot_param_get_double(botparam_, query.c_str() , &max_frequency) == 0){
    std::cout << max_frequency << "Hz \t| " << query_root  << "\n";
  }else{
    max_frequency =1.0;
    std::cout << max_frequency << "Hz \t| " << query_root << " ###### not found. using default ######\n";
  }
  return max_frequency;
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

void joints2frames::publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel){
  if (!shouldPublish(utime, channel) )
    return; 
  
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();  
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();  
  pose_msg.orientation[1] =  r_x.x();  
  pose_msg.orientation[2] =  r_x.y();  
  pose_msg.orientation[3] =  r_x.z();  
  lcm_->publish( channel, &pose_msg);
}


void joints2frames::publishRigidTransform(Eigen::Isometry3d pose, int64_t utime, std::string channel){
  if (!shouldPublish(utime, channel) )
    return; 

  bot_core::rigid_transform_t tf;
  tf.utime = utime;
  tf.trans[0] = pose.translation().x();
  tf.trans[1] = pose.translation().y();
  tf.trans[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  tf.quat[0] = quat.w();
  tf.quat[1] = quat.x();
  tf.quat[2] = quat.y();
  tf.quat[3] = quat.z();
  lcm_->publish(channel, &tf);    
}

// find the channel and check to see if it should be published
// true: publish | false: dont publish
// then update the utime that it was last published at
bool joints2frames::shouldPublish(int64_t utime, std::string channel){
  #if DONT_LIMIT_FREQUENCY
    return true;
  #endif
  
  std::map<string,FrequencyLimit>::iterator it;
  it = pub_frequency_.find( channel );
  
  if(it == pub_frequency_.end()){
    std::cout << channel << " was not found in the pub_frequency list - not publishing\n";
    return false; 
  }
  
  if (utime <  it->second.last_utime ){
    it->second.last_utime  = 0;
    std::cout << utime << " detected negative time change for " << channel << " resetting last_utime\n";
  }
  
  if (utime >  it->second.min_period + it->second.last_utime ){
    it->second.last_utime = utime; 
    return true;
  }
  
  // if published recently - then dont publish again
  return false;
}

void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::robot_state_t* msg){
  
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
  

  // 2. Determine the required BOT_FRAMES transforms:
  for( map<string, KDL::Frame >::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
    std::string link = (*ii).first;
    if (   (*ii).first.compare( "lwr_arm_7_link" ) == 0 ){
      Eigen::Isometry3d body_to_head = KDLToEigen( (*ii).second );
      publishRigidTransform(body_to_head, msg->utime, "BODY_TO_LWR_ARM_7_LINK");
    }
  }

 

  // 3. Loop through joints and extract world positions:
  if (show_triads_){
    int counter =msg->utime;  
    std::vector<Isometry3dTime> body_to_jointTs, world_to_jointsT;
    std::vector< int64_t > body_to_joint_utimes;
    std::vector< std::string > joint_names;
    for( map<string, KDL::Frame >::iterator ii=cartpos_out.begin(); ii!=cartpos_out.end(); ++ii){
      std::string joint = (*ii).first;
      //cout << joint  << ": \n";
      joint_names.push_back( joint  );
      body_to_joint_utimes.push_back( counter);
      
      Eigen::Isometry3d body_to_joint = KDLToEigen( (*ii).second );
      Isometry3dTime body_to_jointT(counter, body_to_joint);
      body_to_jointTs.push_back(body_to_jointT);
      // convert to world positions
      Isometry3dTime world_to_jointT(counter, world_to_body*body_to_joint);
      world_to_jointsT.push_back(world_to_jointT);
      counter++;
    }
    
    pc_vis_->pose_collection_to_lcm_from_list(6001, world_to_jointsT); // all joints in world frame
    if (show_labels_)
      pc_vis_->text_collection_to_lcm(6002, 6001, "Frames [Labels]", joint_names, body_to_joint_utimes );
  }
}


int
main(int argc, char ** argv){
  bool labels = false;
  bool triads = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(triads, "t", "triads","Publish Frame Triads");
  opt.add(labels, "l", "labels","Publish Frame Labels");
  opt.parse();
  if (labels){ // require triads if labels is to be published
    triads=true;
  }
  
  std::cout << "triads: " << triads << "\n";
  std::cout << "labels: " << labels << "\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM("") );
  if(!lcm->good())
    return 1;  
  
  joints2frames app(lcm,labels,triads);
  while(0 == lcm->handle());
  return 0;
}
