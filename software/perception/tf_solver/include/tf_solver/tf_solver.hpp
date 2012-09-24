#ifndef TF_SOLVER_H
#define TF_SOLVER_H


#include <iostream>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include "urdf/model.h"
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"


using namespace std;
using namespace boost;

// ======= usage ===================================
//===============================================
// boost::shared_ptr<forward_kinematics::TfSolver> localTfSolver = boost::shared_ptr<forward_kinematics::TfSolver>(new forward_kinematics::TfSolver(_lcm));
// drc::transform_t rotating_laser_link_tf;
// if(localTfSolver->getLinkTf ("rotating_laser_link",rotating_laser_link_tf)
//  std::cout << "success" <<std::endl;

namespace  forward_kinematics
{

class TfSolver
{
  //--------fields
private:
  std::string _robot_name;
  std::string _urdf_xml_string;
  boost::shared_ptr<lcm::LCM> _lcm;
  boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;
  lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
  std::map<std::string, drc::transform_t > cartpos_out;
  int64_t cartpos_out_utime;

  bool _urdf_parsed;
  bool _urdf_subscription_on;

  //----------------constructor/destructor
public:
  drc::transform_t body_origin_tf; //  current body origin in world frame
  //drc::transform_t rotating_laser_link_tf; // tf of rotating laser link in body frame

  TfSolver(boost::shared_ptr<lcm::LCM> &lcm): _lcm(lcm)
  {
    //lcm ok?
    if(!lcm->good())
    {
      cerr << "\nLCM Not Good: TfSolver" << endl;
      return;
    }

    cartpos_out.clear();
    cartpos_out_utime =0;

    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = lcm->subscribe("ROBOT_MODEL",&TfSolver::handleRobotUrdfMsg,this);
    _urdf_subscription_on = true;

    // Subscribe to Robot_state.
    lcm->subscribe("EST_ROBOT_STATE", &TfSolver::handleRobotStateMsg, this);
  };

  ~TfSolver(){};

  bool getLinkTf(std::string link_name, bot_core::rigid_transform_t &link_tf)
  {
    if (!_urdf_parsed)
    {
      std::cerr<< "URDF not parsed yet\n";
      return 0;
    }

    std::map<std::string, drc::transform_t>::const_iterator it;
    it ==cartpos_out.find(link_name);
    if(it!=cartpos_out.end())// cart pos exists
    {
//      link_tf =cartpos_out.find( link_name)->second;
        drc::transform_t link_tf_old =cartpos_out.find( link_name)->second;
        link_tf.utime =cartpos_out_utime;
        link_tf.trans[0]  = link_tf_old.translation.x;
        link_tf.trans[1]  = link_tf_old.translation.y;
        link_tf.trans[2]  = link_tf_old.translation.z;
        link_tf.quat[0]  = link_tf_old.rotation.w;
        link_tf.quat[1]  = link_tf_old.rotation.x;
        link_tf.quat[2]  = link_tf_old.rotation.y;
        link_tf.quat[3]  = link_tf_old.rotation.z;
      return true;
    }
    else
    {
      std::cerr << "ERROR: TfSolver could not find tf for " << link_name << std::endl;
      return false;
    }

  };



private:

  //-------------message callbacks
  void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
      const std::string& chan,
      const drc::robot_state_t* msg)
  {

    if (!_urdf_parsed)
    {
      return;
    }

    if(_urdf_subscription_on)
    {
      std::cout << "\n TfSolver::handleRobotStateMsg: unsubscribing from _urdf_subscription" << std::endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false;
    }

    KDL::Frame  T_world_body,T_body_world;

    TransformLCMToKDLFrame(msg->origin_position,T_world_body);
    T_body_world=T_world_body.Inverse();
    TransformKDLToLCMFrame(T_body_world,body_origin_tf);

    // calculates the transforms.
    std::map<std::string, double> jointpos_in;
    for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));



    // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin.
    //Otherwise returns relative transforms between joints.

    kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
    cartpos_out_utime = msg->utime;

    if(kinematics_status>=0){
      // cout << "Success!" <<endl;
    }else{
      std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
      return;
    }

    //bool success = getLinkTf("rotating_laser_link",rotating_laser_link_tf);
    //rotating_laser_link_tf =cartpos_out.find("rotating_laser_link")->second;

  };


  void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
      const  drc::robot_urdf_t* msg)
  {

    if(_urdf_parsed ==false) 
    {
      cout<< "\nurdf handler @ RobotStateListener" << endl;
      // Received robot urdf string. Store it internally and get all available joints.
      _robot_name      = msg->robot_name;
      _urdf_xml_string = msg->urdf_xml_string;


      // Get a urdf Model from the xml string and get all the joint names.
      urdf::Model robot_model;
      if (!robot_model.initString( msg->urdf_xml_string))
      {
        std::cerr << "ERROR: Could not generate robot model" << std::endl;
      }


      //---------parse the tree and stop listening for urdf messages

      // Parse KDL tree
      KDL::Tree tree;
      if (!kdl_parser::treeFromString(msg->urdf_xml_string,tree))
      {
        std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
        return;
      }

      //unsubscribe from urdf messages
      //_lcm->unsubscribe(_urdf_subscription);  // crashes viewer if there are other urdf subscriptions in other renderers.

      //
      _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
      //remember that we've parsed the urdf already
      _urdf_parsed = true;

    }//  if(_urdf_parsed ==false)   
  } ;


  void TransformKDLToLCMFrame(const KDL::Frame &k, drc::transform_t &t)
  {
    t.translation.x = k.p[0];
    t.translation.y = k.p[1];
    t.translation.z = k.p[2];

    double x,y,z,w;

    k.M.GetQuaternion(x,y,z,w);
    t.rotation.x =x;
    t.rotation.y =y;
    t.rotation.z =z;
    t.rotation.w =w;
  };

  void TransformLCMToKDLFrame(const drc::transform_t &t, KDL::Frame &k)
  {
    k.p[0] = t.translation.x;
    k.p[1] = t.translation.y;
    k.p[2] = t.translation.z;
    KDL::Rotation M;
    M =  KDL::Rotation::Quaternion( t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
    k.M = M;
  };

  void TransformLCMToKDLFrame(const drc::position_3d_t &t, KDL::Frame &k)
  {
    k.p[0] = t.translation.x;
    k.p[1] = t.translation.y;
    k.p[2] = t.translation.z;
    KDL::Rotation M;
    M =  KDL::Rotation::Quaternion( t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
    k.M = M;
  };


}; //class TfSolver

} //end namespace 


#endif //TF_SOLVER_H

