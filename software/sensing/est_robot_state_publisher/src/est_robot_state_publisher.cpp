#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <map>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"

using namespace std;
using namespace boost;

class RobotStateListener //JointAnglesHandler
{
  //--------fields
private:
  std::string _robot_name;
  std::string _urdf_xml_string;
  std::vector<std::string> _joint_names_;
  std::map<std::string, boost::shared_ptr<urdf::Link> > _links_map;
  boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;

  lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
  bool _urdf_parsed;
  boost::shared_ptr<lcm::LCM> _lcm;

  std::vector<drc::link_transform_t> _link_tfs;


  KDL::Frame  T_world_head;

  //----------------constructor/destructor
public:
  RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm):_urdf_parsed(false),_lcm(lcm) {

    if(!lcm->good())      {
      cerr << "\nLCM Not Good: Robot State Handler" << endl;	return;
    }

    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = lcm->subscribe("ROBOT_MODEL",&RobotStateListener::handleRobotUrdfMsg,this);  

    //lcm->subscribe("MEAS_JOINT_ANGLES", &RobotStateListener::handleJointAnglesMsg, this);  //Subscribes to MEAS_JOINT_ANGLES 
    lcm->subscribe("TRUE_ROBOT_STATE", &RobotStateListener::handleRobotStateMsg, this); //
    lcm->subscribe("POSE_HEAD",&RobotStateListener::handlePoseHeadMsg,this);
    // create subscriptions to contact sensors.

    T_world_head = KDL::Frame::Identity();
  }; // end constructor

  ~RobotStateListener() {};


  //==================================================================================================
  //-------------message callbacks
private:

  //  void handleJointAnglesMsg(const lcm::ReceiveBuffer* rbuf,
  //			       const std::string& chan, 
  //			      const drc::joint_angles_t* msg);
  void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
      const std::string& chan,
      const drc::robot_state_t* TRUE_state_msg)
  {
    if (!_urdf_parsed)
    {
      cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
      return;
    }

    //clear stored data
    _link_tfs.clear();


    // call a routine that calculates the transforms the joint_state_t* msg.
    map<string, double> jointpos_in;
    for (uint i=0; i< (uint) TRUE_state_msg->num_joints; i++) //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(TRUE_state_msg->joint_name[i], TRUE_state_msg->joint_position[i]));

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

    // PRINTS THE VISUAL PROPERTIES OF ALL LINKS THAT HAVE A VISUAL ELEMENT DEFINED IN THE URDF FILE
    map<string, drc::transform_t>::const_iterator transform_it;

    KDL::Frame  T_body_head,T_head_body,T_world_body;

    transform_it=cartpos_out.find("head");

    T_body_head = KDL::Frame::Identity();
    if(transform_it!=cartpos_out.end())// fk cart pos exists
    {

      T_body_head.p[0]= transform_it->second.translation.x;
      T_body_head.p[1]= transform_it->second.translation.y;
      T_body_head.p[2]= transform_it->second.translation.z;
      T_body_head.M =  KDL::Rotation::Quaternion(transform_it->second.rotation.x, transform_it->second.rotation.y, transform_it->second.rotation.z, transform_it->second.rotation.w);

    }
    else{
      std::cout<< "fk position does not exist" <<std::endl;
    }
    
    T_head_body = T_body_head.Inverse();
    
    bot_core::rigid_transform_t tf;
    tf.utime = TRUE_state_msg->utime;
    tf.trans[0] = T_head_body.p[0];
    tf.trans[1] = T_head_body.p[1];
    tf.trans[2] = T_head_body.p[2];
    T_head_body.M.GetQuaternion(tf.quat[1], tf.quat[2], tf.quat[3], tf.quat[0]);
    _lcm->publish("HEAD_TO_BODY", &tf);        
    
    
    
    

    T_world_body  = T_world_head*T_head_body;

    drc::position_3d_t body_origin;
    body_origin.translation.x = T_world_body.p[0];
    body_origin.translation.y = T_world_body.p[1];
    body_origin.translation.z = T_world_body.p[2];
    T_world_body.M.GetQuaternion(body_origin.rotation.x,body_origin.rotation.y,body_origin.rotation.z,body_origin.rotation.w);

    drc::robot_state_t msg;
    msg = *TRUE_state_msg;
    msg.origin_position = body_origin;  //
    _lcm->publish("EST_ROBOT_STATE", &msg);


    
    bot_core::pose_t pose_msg;
    pose_msg.utime = msg.utime;
    pose_msg.pos[0] = body_origin.translation.x;
    pose_msg.pos[1] = body_origin.translation.y;
    pose_msg.pos[2] = body_origin.translation.z;
    pose_msg.orientation[0] = body_origin.rotation.w;
    pose_msg.orientation[1] = body_origin.rotation.x;
    pose_msg.orientation[2] = body_origin.rotation.y;
    pose_msg.orientation[3] = body_origin.rotation.z;
    _lcm->publish("POSE_BODY", &pose_msg); 


    

  };

  //==================================================================================================
  void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
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
    _lcm->unsubscribe(_urdf_subscription); 

    //
    _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
    //remember that we've parsed the urdf already
    _urdf_parsed = true;

    cout<< "Number of Joints: " << _joint_names_.size() <<endl;
  };// end handleRobotUrdfMsg

  //==================================================================================================
  void handlePoseHeadMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
      const  bot_core::pose_t* msg)  {

    // set world_head pose.
    T_world_head.p[0]= msg->pos[0];
    T_world_head.p[1]= msg->pos[1];
    T_world_head.p[2]= msg->pos[2];
    T_world_head.M =  KDL::Rotation::Quaternion(msg->orientation[1],
        msg->orientation[2],
        msg->orientation[3],
        msg->orientation[0]);
    // std::cout<< "head x,y,z in world frame:" <<T_world_head.p[0] <<" , " <<T_world_head.p[1] <<" , "<< T_world_head.p[2] <<std::endl;


  }; // end handleRobotUrdfMsg


}; //end class 


int main (int argc, char ** argv)
{

  boost::shared_ptr<lcm::LCM> _lcm(new lcm::LCM);
  if(!_lcm->good())
    return 1;

  boost::shared_ptr<RobotStateListener> handlerObject(new RobotStateListener(_lcm));

  while(0 == _lcm->handle());

  return 0;
}
