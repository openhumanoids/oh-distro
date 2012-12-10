// LCM example program for interacting  with PCL data
// In this case it pushes the data back out to LCM in
// a different message type for which the "collections"
// renderer can view it in the LCM viewer
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <lcmtypes/visualization.h>


#include "joints2frames.hpp"

using namespace std;
using namespace boost;
using namespace boost::assign;





/////////////////////////////////////

joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm):
          lcm_(publish_lcm), _urdf_parsed(false),
          world_to_bodyT_(0, Eigen::Isometry3d::Identity()),
          body_to_headT_(0, Eigen::Isometry3d::Identity()) {

  j2f_list_ += "head_hokuyo", "left_camera_optical_frame","head_statichokuyo","head_rightedhokuyo";

  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  //pc_vis_->obj_cfg_list.push_back( obj_cfg(6000,"Frames [Zero]",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(6001,"Frames",5,1) );
  // pts: id name type reset objcoll usergb rgb
  
  lcm_->subscribe("EST_ROBOT_STATE",&joints2frames::robot_state_handler,this);  

  _urdf_subscription =   lcm_->subscribe("ROBOT_MODEL",&joints2frames::urdf_handler,this);  
}


void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
  if (!_urdf_parsed){
    cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
    return;
  }

  //clear stored data
  _link_tfs.clear();

  // call a routine that calculates the transforms the joint_state_t* msg.
  map<string, double> jointpos_in;
  for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));

  map<string, drc::transform_t > cartpos_out;

  // Calculate forward position kinematics
  bool kinematics_status;
  bool flatten_tree=true; // determines the absolute transforms with respect to robot origin.
  //Otherwise returns relative transforms between joints.

  kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
  if(kinematics_status>=0){
    // cout << "Success!" <<endl;
  }else{
    cerr << "Error: could not calculate forward kinematics!" <<endl;
    return;
  }
  
  // 1. Extract World Pose:
  world_to_bodyT_.pose.setIdentity();
  world_to_bodyT_.pose.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  world_to_bodyT_.pose.rotate(quat);    
  world_to_bodyT_.utime = msg->utime;
  
  // Loop through joints and extract world positions:
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
    
    // Also see if these joints are required to be published as BOT_FRAMES poses:
    BOOST_FOREACH(string j2f, j2f_list_ ){
      if ( j2f.compare( joint ) == 0 ){
        //cout << "got : " << j2f << "\n";
        bot_core::rigid_transform_t tf;
        // hard coded until inverse kin. is added:
        tf.trans[0] = body_to_joint.translation().x();
        tf.trans[1] = body_to_joint.translation().y();
        tf.trans[2] = body_to_joint.translation().z();
    
        tf.quat[0] =quat.w();
        tf.quat[1] =quat.x();
        tf.quat[2] =quat.y();
        tf.quat[3] =quat.z();
        std::string j2f_upper ="BODY_TO_" + boost::to_upper_copy(j2f);
        lcm_->publish(j2f_upper, &tf);      
      }
    }
    counter++;
  }
  //std::cout << body_to_jointTs.size() << " jts\n";
  //pc_vis_->pose_collection_to_lcm_from_list(6000, body_to_jointTs); // all joints releative to body - publish if necessary
  pc_vis_->pose_collection_to_lcm_from_list(6001, world_to_jointsT); // all joints in world frame
  pc_vis_->text_collection_to_lcm(6002, 6001, "Frames [Labels]", joint_names, body_to_joint_utimes );    
}


void joints2frames::urdf_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
    const  drc::robot_urdf_t* msg)
{
  // Received robot urdf string. Store it internally and get all available joints.
  _robot_name      = msg->robot_name;
  _urdf_xml_string = msg->urdf_xml_string;
  cout<< "\nReceived urdf_xml_string of robot [" 
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
main(int argc, char ** argv)
{
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;  
  
  joints2frames app(lcm);
  while(0 == lcm->handle());
  return 0;
}

