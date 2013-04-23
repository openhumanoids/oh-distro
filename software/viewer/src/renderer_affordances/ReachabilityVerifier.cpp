#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "ReachabilityVerifier.hpp"
#include <algorithm>


using namespace std;
using namespace urdf;
using namespace KDL;
using namespace state;
using namespace kinematics;
using namespace boost;
using namespace visualization_utils;
using namespace collision;

namespace renderer_affordances 
{
  //==================constructor / destructor

  ReachabilityVerifier::ReachabilityVerifier(RendererAffordances* affordance_renderer):
    _parent_renderer(affordance_renderer),_kinematics_model_gfe(1e-3,500) //_kinematics_model_gfe(accuracy,maxiters)
  {
    //cout << "kinematics_model_gfe: " << _kinematics_model_gfe << endl;

  }
 
  ReachabilityVerifier::~ReachabilityVerifier() {

  }

  bool ReachabilityVerifier::has_IK_solution_from_pelvis_to_hand(drc::robot_state_t &statemsg,int hand_type,KDL::Frame &T_world_hand)
  {
  
    State_GFE robot_state;
    robot_state.from_lcm(statemsg);
     _kinematics_model_gfe.set(robot_state);
      
     if(hand_type==0)//SANDIA_LEFT=0, SANDIA_RIGHT=1, 
     { 
       //Frame T_pelvis_world  = _parent_renderer->robotStateListener->T_body_world;
        Frame T_world_pelvis  = _kinematics_model_gfe.link("pelvis");
        Frame pelvis_to_hand_pose = T_world_pelvis.Inverse()*T_world_hand;
 
        State_GFE solution;
        if( _kinematics_model_gfe.inverse_kinematics_left_arm( robot_state, pelvis_to_hand_pose, solution ) ){
          //cout << "found inverse kinematics: " << left_arm_state << endl;
          return true;
        } else {
          //cout << "could not solve inverse kinematics" << endl;
          return false;
        }
      }
      else if(hand_type==1) {
       //Frame T_pelvis_world  = _parent_renderer->robotStateListener->T_body_world;
        Frame T_world_pelvis  = _kinematics_model_gfe.link("pelvis");
        Frame pelvis_to_hand_pose = T_world_pelvis.Inverse()*T_world_hand;
 
        State_GFE solution;
        if( _kinematics_model_gfe.inverse_kinematics_right_arm( robot_state, pelvis_to_hand_pose, solution ) ){
          //cout << "found inverse kinematics: " << right_arm_state << endl;
          return true;
        } else {
          //cout << "could not solve inverse kinematics" << endl;
          return false;
        }
      
      }
      return false;
  
  }
  
  bool ReachabilityVerifier::has_IK_solution_from_pelvis_to_foot(drc::robot_state_t &statemsg,int foot_type,KDL::Frame &T_world_foot)
  {
  
    State_GFE robot_state;
    robot_state.from_lcm(statemsg);
     _kinematics_model_gfe.set(robot_state);
      
     if(foot_type==0)//LEFT=0, RIGHT=1, 
     { 
       //Frame T_pelvis_world  = _parent_renderer->robotStateListener->T_body_world;
        Frame T_world_pelvis  = _kinematics_model_gfe.link("pelvis");
        Frame pelvis_to_foot_pose = T_world_pelvis.Inverse()*T_world_foot;
 
        State_GFE solution;
        if( _kinematics_model_gfe.inverse_kinematics_left_leg( robot_state, pelvis_to_foot_pose, solution ) ){
          //cout << "found inverse kinematics: " << left_leg_state << endl;
          return true;
        } else {
          //cout << "could not solve inverse kinematics" << endl;
          return false;
        }
      }
      else if(foot_type==1) {
       //Frame T_pelvis_world  = _parent_renderer->robotStateListener->T_body_world;
        Frame T_world_pelvis  = _kinematics_model_gfe.link("pelvis");
        Frame pelvis_to_foot_pose = T_world_pelvis.Inverse()*T_world_foot;
 
        State_GFE solution;
        if( _kinematics_model_gfe.inverse_kinematics_right_leg( robot_state, pelvis_to_foot_pose, solution ) ){
          //cout << "found inverse kinematics: " << right_leg_state << endl;
          return true;
        } else {
          //cout << "could not solve inverse kinematics" << endl;
          return false;
        }
      }
      return false;
  }

//------------------------------------------------------------------------------ 

  


} //end namespace


