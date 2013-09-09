#ifndef AFFORDANCE_SEED_UTILS_HPP
#define AFFORDANCE_SEED_UTILS_HPP
#include <lcmtypes/drc_lcmtypes.hpp>
#include <algorithm>
#include <string> 
#include <visualization_utils/eigen_kdl_conversions.hpp>

using namespace std;
using namespace boost;
//using namespace visualization_utils;

namespace visualization_utils
{
//// Preparation Function Before Encoding To XML.
inline static void prepareKeyframePlanForStorage(KDL::Frame &T_world_aff,
                                     drc::robot_plan_w_keyframes_t &msg_in,
                                     std::vector<std::string> &stateframe_ids,
                                     std::vector< std::vector<double> > &stateframe_values,
                                     std::vector<std::string> &graspframe_ids,
                                     std::vector< std::vector<double> > &graspframe_values)
{ 

    stateframe_ids.clear();
    stateframe_values.clear();
    graspframe_ids.clear();
    graspframe_values.clear();
    for (uint i = 0; i <(uint)msg_in.num_states; i++)
    {

      if(i==0)
      {
        stateframe_ids.push_back("is_keyframe");  
        stateframe_ids.push_back("is_breakpoint");
        stateframe_ids.push_back("utime");
        stateframe_ids.push_back("pelvis_x");  
        stateframe_ids.push_back("pelvis_y");
        stateframe_ids.push_back("pelvis_z");
        stateframe_ids.push_back("pelvis_qw");
        stateframe_ids.push_back("pelvis_qx");
        stateframe_ids.push_back("pelvis_qy");
        stateframe_ids.push_back("pelvis_qz");
      }// end if  
      
      std::vector<double> temp;
      temp.push_back(msg_in.is_keyframe[i]);
      temp.push_back(msg_in.is_breakpoint[i]);    
      temp.push_back(msg_in.plan[i].utime);
      //convert pose from world frame to affordance frame here.
      KDL::Frame T_world_oldpose, T_aff_newpose;
      drc::position_3d_t newpose;
      visualization_utils::transformLCMToKDL(msg_in.plan[i].pose,T_world_oldpose);
      T_aff_newpose =(T_world_aff.Inverse())*T_world_oldpose;
      visualization_utils::transformKDLToLCM(T_aff_newpose,newpose);

      temp.push_back(newpose.translation.x);
      temp.push_back(newpose.translation.y);
      temp.push_back(newpose.translation.z);
      temp.push_back(newpose.rotation.w);
      temp.push_back(newpose.rotation.x);
      temp.push_back(newpose.rotation.y);
      temp.push_back(newpose.rotation.z);     
      for (uint k = 0; k <(uint)msg_in.plan[i].num_joints; k++)
      {  
        if(i==0)
          stateframe_ids.push_back(msg_in.plan[i].joint_name[k]);
        temp.push_back(msg_in.plan[i].joint_position[k]);
      }
       stateframe_values.push_back(temp);
    }// end for

    for (uint i = 0; i <(uint)msg_in.num_grasp_transitions; i++)
    {
      if(i==0)
      {
        graspframe_ids.push_back("grasp_on");
        graspframe_ids.push_back("grasp_type");
        graspframe_ids.push_back("power_grasp");
        graspframe_ids.push_back("utime");
        graspframe_ids.push_back("hand_x");  
        graspframe_ids.push_back("hand_y");
        graspframe_ids.push_back("hand_z");
        graspframe_ids.push_back("hand_qw");
        graspframe_ids.push_back("hand_qx");
        graspframe_ids.push_back("hand_qy");
        graspframe_ids.push_back("hand_qz");
      }// end if   
      std::vector<double> temp;    
      temp.push_back(msg_in.grasps[i].grasp_on);
      temp.push_back(msg_in.grasps[i].grasp_type);
      temp.push_back(msg_in.grasps[i].power_grasp);
      temp.push_back(msg_in.grasps[i].utime);
      //convert hand pose to affordance frame here.
      KDL::Frame T_world_oldpose, T_aff_newpose;
      drc::position_3d_t newpose;
      visualization_utils::transformLCMToKDL(msg_in.grasps[i].hand_pose,T_world_oldpose);
      T_aff_newpose =(T_world_aff.Inverse())*T_world_oldpose;
      visualization_utils::transformKDLToLCM(T_aff_newpose,newpose); 
      temp.push_back(newpose.translation.x);
      temp.push_back(newpose.translation.y);
      temp.push_back(newpose.translation.z);
      temp.push_back(newpose.rotation.w);
      temp.push_back(newpose.rotation.x);
      temp.push_back(newpose.rotation.y);
      temp.push_back(newpose.rotation.z); 
      for (uint k = 0; k <(uint)msg_in.grasps[i].num_joints; k++)
      {  
        if(i==0)
          graspframe_ids.push_back(msg_in.grasps[i].joint_name[k]);
        temp.push_back(msg_in.grasps[i].joint_position[k]);
      }
      graspframe_values.push_back(temp); 
    }// end for  

 }

  ////////////////////////////////////////////////////////////////////////////////////////////////
 static bool getField(std::vector<std::string> &ids,std::vector<double> &values,const std::string &field, double &value)
 {
      unsigned int index;
      std::vector<std::string>::const_iterator found;
      found = std::find (ids.begin(), ids.end(), field); 
      if(found ==  ids.end()) 
      {
       cout << field << " field not found, could not decode stored keyframe plan " << endl;
       return false;
      }
      index = found - ids.begin();
      value = values[index];
      return true;
 }
 
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //// Decode to robot_plan_w_keyframes_t  given PlanSeed
  inline static void decodeKeyframePlanFromStorage(KDL::Frame &T_world_aff,
                                   std::vector<std::string> &stateframe_ids,
                                   std::vector< std::vector<double> > &stateframe_values,
                                   std::vector<std::string> &graspframe_ids,
                                   std::vector< std::vector<double> > &graspframe_values,
                                   drc::robot_plan_w_keyframes_t &msg_out)
  {
    msg_out.utime = bot_timestamp_now();
    msg_out.robot_name = " ";
    msg_out.num_states = stateframe_values.size();
    msg_out.num_bytes =0;
    msg_out.num_keyframes = 0;
    msg_out.num_breakpoints = 0;
    for (uint i = 0; i <(uint)stateframe_values.size(); i++) // N frames = N States
    {    


      std::string field;
      unsigned int index;
      
      drc::robot_state_t state_msg;
      
      double value;
      if(getField(stateframe_ids,stateframe_values[i],"is_keyframe",value)){
        msg_out.is_keyframe.push_back((value==1.0));
        msg_out.num_keyframes += (int)(value==1.0);
      }
      else
        return;
        
      if(getField(stateframe_ids,stateframe_values[i],"is_breakpoint",value)){
        msg_out.is_breakpoint.push_back((value==1.0));
        msg_out.num_breakpoints += (int)(value==1.0);
      }
      else
        return;

      if(getField(stateframe_ids,stateframe_values[i],"utime",value)){
        state_msg.utime = value;
      }
      else
        return;
        
      drc::position_3d_t storedpose;

      if(getField(stateframe_ids,stateframe_values[i],"pelvis_x",value))
        storedpose.translation.x = value;
      else
        return;

      if(getField(stateframe_ids,stateframe_values[i],"pelvis_y",value))
        storedpose.translation.y = value;   
      else
        return; 
             
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_z",value))
        storedpose.translation.z = value;   
      else
        return;    
      
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qw",value))
        storedpose.rotation.w = value;
      else
        return;

      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qx",value))
        storedpose.rotation.x = value;   
      else
        return; 
             
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qy",value))
        storedpose.rotation.y = value;   
      else
        return; 
         
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qz",value))
        storedpose.rotation.z = value;   
      else
        return; 
        
      // note: can lose quat normalization in storage. 
	  // transformLCMToKDL makes sure that storepose quaternion is normalized 
       
        
      KDL::Frame T_aff_oldpose, T_world_newpose;
      drc::position_3d_t newpose;
      visualization_utils::transformLCMToKDL(storedpose,T_aff_oldpose);
      T_world_newpose = T_world_aff*T_aff_oldpose;
      visualization_utils::transformKDLToLCM(T_world_newpose,newpose); 
      state_msg.pose=newpose;
     
      state_msg.num_joints = stateframe_ids.size()-10;          
      for (uint k = 10; k <(uint)stateframe_ids.size(); k++)
      {  
        state_msg.joint_name.push_back(stateframe_ids[k]);
        state_msg.joint_position.push_back(stateframe_values[i][k]);
        state_msg.joint_velocity.push_back(0);
        state_msg.joint_effort.push_back(0);
      }
      state_msg.force_torque = drc::force_torque_t();
      msg_out.plan.push_back(state_msg);
    }// end for stateframe_values.size() 
    
    
    msg_out.num_grasp_transitions = graspframe_values.size();
    for (uint i = 0; i <(uint)graspframe_values.size(); i++) // M Graspframes = M GraspTransition
    {    

      std::string field;
      unsigned int index;
      
      drc::grasp_transition_state_t graspstate_msg;

      double value;
      if(getField(graspframe_ids,graspframe_values[i],"grasp_on",value))
        graspstate_msg.grasp_on =(value==1.0);
      else
        return;
        
      if(getField(graspframe_ids,graspframe_values[i],"grasp_type",value))
        graspstate_msg.grasp_type =value;
      else
        return;
        
      if(getField(graspframe_ids,graspframe_values[i],"power_grasp",value))
         graspstate_msg.power_grasp =(value==1.0);
      else
        return;
        
      if(getField(graspframe_ids,graspframe_values[i],"utime",value)){
        graspstate_msg.utime = value;
      }
      else
        return;   
        
      drc::position_3d_t storedpose;

      if(getField(graspframe_ids,graspframe_values[i],"hand_x",value))
        storedpose.translation.x = value;
      else
        return;

      if(getField(graspframe_ids,graspframe_values[i],"hand_y",value))
        storedpose.translation.y = value;   
      else
        return; 
             
      if(getField(graspframe_ids,graspframe_values[i],"hand_z",value))
        storedpose.translation.z = value;   
      else
        return;    
      
      if(getField(graspframe_ids,graspframe_values[i],"hand_qw",value))
        storedpose.rotation.w = value;
      else
        return;

      if(getField(graspframe_ids,graspframe_values[i],"hand_qx",value))
        storedpose.rotation.x = value;   
      else
        return; 
             
      if(getField(graspframe_ids,graspframe_values[i],"hand_qy",value))
        storedpose.rotation.y = value;   
      else
        return; 
         
      if(getField(graspframe_ids,graspframe_values[i],"hand_qz",value))
        storedpose.rotation.z = value;   
      else
        return; 
        
      KDL::Frame T_aff_oldpose, T_world_newpose;
      drc::position_3d_t newpose;
      visualization_utils::transformLCMToKDL(storedpose,T_aff_oldpose);
      T_world_newpose = T_world_aff*T_aff_oldpose;
      visualization_utils::transformKDLToLCM(T_world_newpose,newpose); 
      graspstate_msg.hand_pose=newpose;
     
      graspstate_msg.num_joints = graspframe_ids.size()-11;          
      for (uint k = 11; k <(uint)graspframe_ids.size(); k++)
      {  
        graspstate_msg.joint_name.push_back(graspframe_ids[k]);
        graspstate_msg.joint_position.push_back(graspframe_values[i][k]);
      }
      
      msg_out.grasps.push_back(graspstate_msg);
    }// end for graspstate_values.size()     
    
    
  } // end function
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //// Decode first pose to robot_state_t  given PlanSeed (To be used as a posture goal)
  inline static void decodeAndExtractFirstFrameInKeyframePlanFromStorage(KDL::Frame &T_world_aff,
                                   std::vector<std::string> &stateframe_ids,
                                   std::vector< std::vector<double> > &stateframe_values,
                                   drc::robot_state_t &msg_out)
  {
      msg_out.utime = bot_timestamp_now();

      uint i = 0;//(uint)stateframe_values.size()-1;

      std::string field;
      unsigned int index;
      
      drc::robot_state_t state_msg;
      
      double value;
        
      drc::position_3d_t storedpose;

      if(getField(stateframe_ids,stateframe_values[i],"pelvis_x",value))
        storedpose.translation.x = value;
      else
        return;

      if(getField(stateframe_ids,stateframe_values[i],"pelvis_y",value))
        storedpose.translation.y = value;   
      else
        return; 
             
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_z",value))
        storedpose.translation.z = value;   
      else
        return;    
      
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qw",value))
        storedpose.rotation.w = value;
      else
        return;

      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qx",value))
        storedpose.rotation.x = value;   
      else
        return; 
             
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qy",value))
        storedpose.rotation.y = value;   
      else
        return; 
         
      if(getField(stateframe_ids,stateframe_values[i],"pelvis_qz",value))
        storedpose.rotation.z = value;   
      else
        return; 
        
      KDL::Frame T_aff_oldpose, T_world_newpose;
      drc::position_3d_t newpose;
      visualization_utils::transformLCMToKDL(storedpose,T_aff_oldpose);
      T_world_newpose = T_world_aff*T_aff_oldpose;
      visualization_utils::transformKDLToLCM(T_world_newpose,newpose); 
      msg_out.pose=newpose;
     
      msg_out.num_joints = stateframe_ids.size()-10;          
      for (uint k = 10; k <(uint)stateframe_ids.size(); k++)
      {  
        msg_out.joint_name.push_back(stateframe_ids[k]);
        msg_out.joint_position.push_back(stateframe_values[i][k]);
        msg_out.joint_velocity.push_back(0);
        msg_out.joint_effort.push_back(0);
      }
      msg_out.force_torque = drc::force_torque_t();
     
  } //end function

  ////////////////////////////////////////////////////////////////////////////////////////////////
  //// Decode first pose given PlanSeed and publish as a posture goal       
  inline static void getFirstFrameInPlanAsPostureGoal(KDL::Frame &T_world_aff,
                                   std::vector<std::string> &stateframe_ids,
                                   std::vector< std::vector<double> > &stateframe_values,
                                  drc::joint_angles_t &posture_goal_msg)
  {
      drc::robot_state_t msg;      
                       
      decodeAndExtractFirstFrameInKeyframePlanFromStorage(T_world_aff,
                                                          stateframe_ids,
                                                          stateframe_values,
                                                            msg);
      posture_goal_msg.utime=msg.utime;
      posture_goal_msg.robot_name=" ";
      posture_goal_msg.num_joints=msg.num_joints;
      for (uint k = 0; k <(uint)msg.num_joints; k++)
      {  
       posture_goal_msg.joint_name.push_back(msg.joint_name[k]);
       posture_goal_msg.joint_position.push_back((double)msg.joint_position[k]);
      }
  }      

}// end namespace

#endif //AFFORDANCE_SEED_UTILS_HPP
