#ifndef  VICON_KINEMATICS_HPP
#define  VICON_KINEMATICS_HPP

#include <iostream>
#include <map>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "urdf/model.h"


namespace vicon_kinematics
{


typedef struct _point3d {
    double x;
    double y;
    double z;
} point3d_t;

/*typedef struct _atlasJointAngles {
    double LShoulderPitch;
    double LShoulderRoll;
    double LElbowRoll;
    double LElbowPitch;
    double LWristRoll;
    double LWristPitch;
    double RShoulderPitch;
    double RShoulderRoll;
    double RElbowRoll;
    double RElbowPitch;
    double RWristRoll;
    double RWristPitch;
    double NeckPitch;
} atlasJointAngles_t;*/

typedef struct _atlasJointAngles {
    double LShoulderYaw;
    double LShoulderPitch;
    double LShoulderRoll;
    double LElbowPitch;
    double LWristYaw;
    double LWristRoll;
    double RShoulderYaw;
    double RShoulderPitch;
    double RShoulderRoll;
    double RElbowPitch;
    double RWristYaw;
    double RWristRoll;
    double NeckYaw;
    double NeckPitch;
} atlasJointAngles_t;

typedef struct _fourMarkers {
    point3d_t LEndeffector;
    point3d_t REndeffector;
    point3d_t LShoulder;
    point3d_t RShoulder;
} fourMarkers_t;

  class ViconKinematicsSolver
  {
    //--------fields
  private:
    std::string _robot_name;
    std::string _urdf_xml_string; 
    std::vector<std::string> _joint_names_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;
    
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
 
    boost::shared_ptr<lcm::LCM> _lcm;    

    
    bool _urdf_parsed;
    bool _urdf_subscription_on;

    //----------------constructor/destructor
  public:
    ViconKinematicsSolver(boost::shared_ptr<lcm::LCM> &lcm): _lcm(lcm)
    {
    
      //lcm ok?
      if(!_lcm->good())
        {
            std::cerr << "\nLCM Not Good: TfSolver" << std::endl;
            return;
        }
        
      // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
      _urdf_subscription = _lcm->subscribe("ROBOT_MODEL",&vicon_kinematics::ViconKinematicsSolver::handleRobotUrdfMsg,this);    
      _urdf_subscription_on = true;
      _urdf_parsed= false;
      
      // Also subscribe to vicon msgs.
      //lcm->subscribe("VICON_MSGS", &TfSolver::handleRobotStateMsg, this); 
    };
    ~ViconKinematicsSolver(){};
    
    
     //-------------message callback
  private:

  void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,  const  drc::robot_urdf_t* msg) 
  {

    if(_urdf_parsed==false) 
    {
       std::cout<< "\nurdf handler @ RobotStateListener" << std::endl;
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
          }

        //unsubscribe from urdf messages
        

        std::cout << "\n handleRobotStateMsg in vicon_kinematics: unsubscribing from _urdf_subscription" << std::endl;
        _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
          _urdf_subscription_on =  false;     
    
        //
        _fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
        //remember that we've parsed the urdf already
        _urdf_parsed = true;
    
     }//  if(_urdf_parsed ==false)  
    
  };
  
  public:   
    
  void publish_ee_goals_given_vicon_angles (atlasJointAngles_t &joint_angles)
  {
  
      if (!_urdf_parsed)
        {
            return;
        }

    //std::cout<< "publishing ee_goals given vicon angles" <<std::endl;
     // perform Forward Kinematics given robot model and vicon joint angles.
      std::map<std::string,double> jointpos_in;
     jointpos_in.insert(std::make_pair("LShoulderYaw", joint_angles.LShoulderYaw));
     jointpos_in.insert(std::make_pair("LShoulderPitch", joint_angles.LShoulderPitch));
     jointpos_in.insert(std::make_pair("LShoulderRoll", joint_angles.LShoulderRoll));
     jointpos_in.insert(std::make_pair("LElbowPitch", joint_angles.LElbowPitch));
     jointpos_in.insert(std::make_pair("LWristYaw", joint_angles.LWristYaw));
     jointpos_in.insert(std::make_pair("LWristRoll", joint_angles.LWristRoll));
     jointpos_in.insert(std::make_pair("RShoulderYaw", joint_angles.RShoulderYaw));
     jointpos_in.insert(std::make_pair("RShoulderPitch", joint_angles.RShoulderPitch));
     jointpos_in.insert(std::make_pair("RShoulderRoll", joint_angles.RShoulderRoll));
     jointpos_in.insert(std::make_pair("RElbowPitch", joint_angles.RElbowPitch));
     jointpos_in.insert(std::make_pair("RWristYaw", joint_angles.RWristYaw));
     jointpos_in.insert(std::make_pair("RWristRoll", joint_angles.RWristRoll));
     jointpos_in.insert(std::make_pair("NeckYaw", joint_angles.NeckYaw));
     jointpos_in.insert(std::make_pair("NeckPitch", joint_angles.NeckPitch));

     
     //forward kinematics given robot model
     
     std::map<std::string, drc::transform_t > cartpos_out;
        
        // Calculate forward position kinematics
      bool kinematics_status;
      bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                                //Otherwise returns relative transforms between joints. 
      kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
      if(kinematics_status>=0){
          // std::cout << "Success!" <<std::endl;
       }else{
           std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
           return;
       }
     
     std::map<std::string,double> L_null_space_bias;
     L_null_space_bias.insert(std::make_pair("LShoulderYaw", joint_angles.LShoulderYaw));
     L_null_space_bias.insert(std::make_pair("LShoulderPitch", joint_angles.LShoulderPitch));
     L_null_space_bias.insert(std::make_pair("LShoulderRoll", joint_angles.LShoulderRoll));
     L_null_space_bias.insert(std::make_pair("LElbowPitch", joint_angles.LElbowPitch));
     L_null_space_bias.insert(std::make_pair("LWristYaw", joint_angles.LWristYaw));
     L_null_space_bias.insert(std::make_pair("LWristRoll", joint_angles.LWristRoll));
     std::map<std::string,double> R_null_space_bias;
     R_null_space_bias.insert(std::make_pair("RShoulderYaw", joint_angles.RShoulderYaw));
     R_null_space_bias.insert(std::make_pair("RShoulderPitch", joint_angles.RShoulderPitch));
     R_null_space_bias.insert(std::make_pair("RShoulderRoll", joint_angles.RShoulderRoll));
     R_null_space_bias.insert(std::make_pair("RElbowPitch", joint_angles.RElbowPitch));
     R_null_space_bias.insert(std::make_pair("RWristYaw", joint_angles.RWristYaw));
     R_null_space_bias.insert(std::make_pair("RWristRoll", joint_angles.RWristRoll));
     
     KDL::Frame  T_body_Lendeffector,T_body_Rendeffector;

     drc::transform_t LWristRoll_link_tf =cartpos_out.find("LWristRoll_link")->second;
     drc::transform_t RWristRoll_link_tf =cartpos_out.find("RWristRoll_link")->second;
     TransformLCMToKDLFrame(LWristRoll_link_tf,T_body_Lendeffector);
     TransformLCMToKDLFrame(RWristRoll_link_tf,T_body_Rendeffector);

     publish_eegoal_with_bias(T_body_Lendeffector,L_null_space_bias, "LWRISTROLL_LINK_GOAL");
     publish_eegoal_with_bias(T_body_Rendeffector,R_null_space_bias, "RWRISTROLL_LINK_GOAL");
     //publish_eegoal(T_body_Lendeffector, "LWRISTROLL_LINK_GOAL");
     //publish_eegoal(T_body_Lendeffector, "RWRISTROLL_LINK_GOAL");
    };
    
  void publish_ee_goals_given_four_vicon_markers(fourMarkers_t &vicon_markers) 
  {

    printf("inside publish_ee_goals_given_four_vicon_markers 1 ");
  
      if (!_urdf_parsed)
        {
            return;
        }

printf("inside publish_ee_goals_given_four_vicon_markers 2");


//std::cout<< "publishing ee_goals given four vicon markers" <<std::endl;
  
 // from forward kinematics
 KDL::Frame  T_body_Lshoulder, T_body_Rshoulder; // shoulder positions in body frame
 
  // shoulders are fixed relative to body origin for wheeled_atlas model.
  T_body_Lshoulder.p[0]= 0.0;
    T_body_Lshoulder.p[1]= 0.241059;
    T_body_Lshoulder.p[2]= 0.964264;
    T_body_Lshoulder.M =  KDL::Rotation::RPY(0.0,0.0,0.0); // can vicon give orientation? you might need two markers.
 
  T_body_Rshoulder.p[0]= 0.0;
    T_body_Rshoulder.p[1]= -0.241059;
    T_body_Rshoulder.p[2]= 0.964264;
    T_body_Rshoulder.M =  KDL::Rotation::RPY(0.0,0.0,0.0); // can vicon give orientation? you might need two markers.

  
 // from vicon 
 KDL::Frame  T_world_Lendeffector, T_world_Rendeffector;
      
  T_world_Lendeffector.p[0]= vicon_markers.LEndeffector.x;
    T_world_Lendeffector.p[1]= vicon_markers.LEndeffector.y;
    T_world_Lendeffector.p[2]= vicon_markers.LEndeffector.z;
    T_world_Lendeffector.M =  KDL::Rotation::RPY(0.0,0.0,0.0); // can vicon give orientation? you might need two markers.
    
    T_world_Rendeffector.p[0]= vicon_markers.REndeffector.x;
    T_world_Rendeffector.p[1]= vicon_markers.REndeffector.y;
    T_world_Rendeffector.p[2]= vicon_markers.REndeffector.z;
    T_world_Rendeffector.M =  KDL::Rotation::RPY(0.0,0.0,0.0);
    
 KDL::Frame  T_world_Lshoulder, T_world_Rshoulder;
 
  T_world_Lshoulder.p[0]= vicon_markers.LShoulder.x;
    T_world_Lshoulder.p[1]= vicon_markers.LShoulder.y;
    T_world_Lshoulder.p[2]= vicon_markers.LShoulder.z;
    T_world_Lshoulder.M =  KDL::Rotation::RPY(0*(M_PI/180),0*(M_PI/180),0.0);
    
    T_world_Rshoulder.p[0]= vicon_markers.RShoulder.x;
    T_world_Rshoulder.p[1]= vicon_markers.RShoulder.y;
    T_world_Rshoulder.p[2]= vicon_markers.RShoulder.z;
    T_world_Rshoulder.M =  KDL::Rotation::RPY(0*(M_PI/180),0*(M_PI/180),0.0);
    
    
 KDL::Frame  T_Lshoulder_world, T_Rshoulder_world;
 T_Lshoulder_world = T_world_Lshoulder.Inverse(); 
 T_Rshoulder_world = T_world_Rshoulder.Inverse(); 
 
 KDL::Frame  T_Lshoulder_Lendeffector, T_Rshoulder_Rendeffector; 
 T_Lshoulder_Lendeffector  = T_Lshoulder_world * T_world_Lendeffector;
 T_Rshoulder_Rendeffector  = T_Rshoulder_world * T_world_Rendeffector;
 
 
 KDL::Frame  T_body_Lendeffector,T_body_Rendeffector;
 T_body_Lendeffector = T_body_Lshoulder*T_Lshoulder_Lendeffector;
 T_body_Rendeffector = T_body_Rshoulder*T_Rshoulder_Rendeffector;
 
 publish_eegoal(T_body_Lendeffector, "LWRISTROLL_LINK_GOAL");
 publish_eegoal(T_body_Rendeffector, "RWRISTROLL_LINK_GOAL");

 };
 //-----------
    
    

  //============================
  void publish_eegoal_with_bias( KDL::Frame &T_body_ee, std::map<std::string,double> &null_space_bias, std::string channel)
   {
    drc::ee_goal_t goalmsg;
    
    double x,y,z,w;
      T_body_ee.M.GetQuaternion(x,y,z,w);

      goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
      goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
      goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

      goalmsg.ee_goal_pos.rotation.x = x;
      goalmsg.ee_goal_pos.rotation.y = y;
      goalmsg.ee_goal_pos.rotation.z = z;
      goalmsg.ee_goal_pos.rotation.w = w;

      goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
      goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
      goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
      goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
      goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
      goalmsg.ee_goal_twist.angular_velocity.z = 0.0;
    
      goalmsg.num_chain_joints  = 6;

//      goalmsg.use_posture_bias  = false;
//      goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
//      goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
//      for(int i = 0; i < goalmsg.num_chain_joints; i++){
//          goalmsg.joint_posture_bias[i]=0;
//          goalmsg.chain_joint_names[i]= "dummy";
//      }

    goalmsg.use_posture_bias  = true;
    typedef std::map<std::string, double > jointAnglesType;
  for( jointAnglesType::const_iterator it = null_space_bias.begin(); it!=null_space_bias.end(); it++)
  { 
    goalmsg.chain_joint_names.push_back(it->first);
    goalmsg.joint_posture_bias.push_back(it->second);
  }

      // Publish the message
      goalmsg.halt_ee_controller = false;

    _lcm->publish(channel, &goalmsg);
   };
   
   
   
   //-----------
   void publish_eegoal( KDL::Frame &T_body_ee, std::string channel)
   {
    drc::ee_goal_t goalmsg;
    
    double x,y,z,w;
      T_body_ee.M.GetQuaternion(x,y,z,w);

      goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
      goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
      goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

      goalmsg.ee_goal_pos.rotation.x = x;
      goalmsg.ee_goal_pos.rotation.y = y;
      goalmsg.ee_goal_pos.rotation.z = z;
      goalmsg.ee_goal_pos.rotation.w = w;

      goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
      goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
      goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
      goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
      goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
      goalmsg.ee_goal_twist.angular_velocity.z = 0.0;
    
      goalmsg.num_chain_joints  = 6;
      // No specified posture bias
      goalmsg.use_posture_bias  = false;
      goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
      goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
      for(int i = 0; i < goalmsg.num_chain_joints; i++){
          goalmsg.joint_posture_bias[i]=0;
          goalmsg.chain_joint_names[i]= "dummy_joint_names";
      }

      // Publish the message
      goalmsg.halt_ee_controller = false;

    _lcm->publish(channel, &goalmsg);
   };

 //-----------  
  
   
  
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
    


}; //class ViconKinematicsSolver

} //namespace vicon_kinematics


#endif //VICON_KINEMATICS_HPP
