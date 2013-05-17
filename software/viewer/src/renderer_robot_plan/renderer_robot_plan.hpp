#ifndef RENDERER_ROBOTPLAN_HPP
#define RENDERER_ROBOTPLAN_HPP

#include <iostream>
#include <boost/function.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "lcmtypes/drc_lcmtypes.h"

#include <GL/gl.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_core/rotations.h>
#include <gdk/gdkkeysyms.h>
#include <Eigen/Dense>

#include <visualization_utils/angles.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;
//using namespace renderer_robot_plan;

namespace renderer_robot_plan 
{

  typedef struct _RendererRobotPlan
  {
    BotRenderer renderer;
    BotViewer          *viewer;
    BotGtkParamWidget *pw;
    boost::shared_ptr<RobotPlanListener> robotPlanListener;
    boost::shared_ptr<lcm::LCM> lcm;
    int64_t max_draw_utime;
    BotEventHandler ehandler;
    bool selection_enabled;
    bool clicked;
    bool dragging;  
    bool visualize_bbox;
    bool use_colormap;
    bool adjust_endstate;
    bool show_fullplan;
    Eigen::Vector3f ray_start;
    Eigen::Vector3f ray_end;
    Eigen::Vector3f ray_hit;
    Eigen::Vector3f ray_hit_drag;
    Eigen::Vector3f prev_ray_hit_drag;
    Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
    double ray_hit_t;
    
    
    
    std::string* selection;
    std::string* marker_selection;
    bool is_left_in_motion;
    bool is_hand_in_motion;
    uint selected_plan_index;
    uint selected_keyframe_index;
    uint displayed_plan_index;
    // Our only source of a free running clock:
    int64_t robot_utime;
    GtkWidget *plan_execution_dock;
    GtkWidget *plan_approval_dock;
    
    // Vicon seed planning collection settings:
    int vicon_n_plan_samples;
    double vicon_sample_period;
    int8_t vicon_type;
    
  } RendererRobotPlan;
  
// ===================================================================
// Distance queries
// ===================================================================
  inline static double get_shortest_distance_between_keyframes_and_markers (void *user,Eigen::Vector3f &from,Eigen::Vector3f &to)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    collision::Collision_Object * intersected_object = NULL;
    Eigen::Vector3f hit_pt;
    double shortest_distance = -1;
  
       if((self->robotPlanListener->_gl_left_hand)&&(self->robotPlanListener->_gl_right_hand)
        &&(self->robotPlanListener->_gl_left_foot)&&(self->robotPlanListener->_gl_right_foot)) // exists
      {
        if(self->robotPlanListener->_gl_left_hand->is_bodypose_adjustment_enabled())
        {
        
            self->robotPlanListener->_gl_left_hand->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
            
            if(intersected_object != NULL )
            {
                self->ray_hit = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                Eigen::Vector3f diff = (from-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    self->ray_hit = hit_pt;
                    self->ray_hit_drag = hit_pt;
                    self->ray_hit_t = (hit_pt - self->ray_start).norm();
                    //(*self->selection)  =  ;
                    (*self->marker_selection)  = string(intersected_object->id().c_str());
                     self->is_left_in_motion =  true;
                     self->is_hand_in_motion =  true;
                  }
                }
                else {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  //(*self->selection)  =  ;
                  (*self->marker_selection)  = string(intersected_object->id().c_str());
                   self->is_left_in_motion =  true;
                   self->is_hand_in_motion =  true;
                 }
            }
            else 
            {
              // clear previous selections
              string no_selection = " ";
              self->robotPlanListener->_gl_left_hand->highlight_link(no_selection); 
            }  // end if-else intersected_object !=NULL;
                       
        }// end if(self->robotPlanListener->_gl_left_hand->is_bodypose_adjustment_enabled())
        
        if(self->robotPlanListener->_gl_right_hand->is_bodypose_adjustment_enabled())
        {
      
            self->robotPlanListener->_gl_right_hand->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
          
            if(intersected_object != NULL )
            {
                self->ray_hit = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                Eigen::Vector3f diff = (from-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    self->ray_hit = hit_pt;
                    self->ray_hit_drag = hit_pt;
                    self->ray_hit_t = (hit_pt - self->ray_start).norm();
                    //(*self->selection)  =  ;
                    (*self->marker_selection)  = string(intersected_object->id().c_str());
                    self->is_left_in_motion =  false;
                    self->is_hand_in_motion =  true;
                  }
                }
                else {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  //(*self->selection)  =  ;
                  (*self->marker_selection)  = string(intersected_object->id().c_str());
                  self->is_left_in_motion =  false;
                  self->is_hand_in_motion =  true;
                 }
            }
            else 
            {
              // clear previous selections
              string no_selection = " ";
              self->robotPlanListener->_gl_right_hand->highlight_link(no_selection); 
            }  // end if-else intersected_object !=NULL;
                       
        }// end if(self->robotPlanListener->_gl_right_hand->is_bodypose_adjustment_enabled())
 
       if(self->robotPlanListener->_gl_left_foot->is_bodypose_adjustment_enabled())
        {
        
            self->robotPlanListener->_gl_left_foot->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
            
            if(intersected_object != NULL )
            {
                self->ray_hit = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                Eigen::Vector3f diff = (from-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    self->ray_hit = hit_pt;
                    self->ray_hit_drag = hit_pt;
                    self->ray_hit_t = (hit_pt - self->ray_start).norm();
                    //(*self->selection)  =  ;
                    (*self->marker_selection)  = string(intersected_object->id().c_str());
                     self->is_left_in_motion =  true;
                     self->is_hand_in_motion =  false;
                  }
                }
                else {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  //(*self->selection)  =  ;
                  (*self->marker_selection)  = string(intersected_object->id().c_str());
                   self->is_left_in_motion =  true;
                   self->is_hand_in_motion =  false;
                 }
            }
            else 
            {
              // clear previous selections
              string no_selection = " ";
              self->robotPlanListener->_gl_left_foot->highlight_link(no_selection); 
            }  // end if-else intersected_object !=NULL;
                       
        }// end if(self->robotPlanListener->_gl_left_foot->is_bodypose_adjustment_enabled())       
        if(self->robotPlanListener->_gl_right_foot->is_bodypose_adjustment_enabled())
        {
      
            self->robotPlanListener->_gl_right_foot->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
          
            if(intersected_object != NULL )
            {
                self->ray_hit = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                Eigen::Vector3f diff = (from-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                  {
                    shortest_distance = distance;
                    self->ray_hit = hit_pt;
                    self->ray_hit_drag = hit_pt;
                    self->ray_hit_t = (hit_pt - self->ray_start).norm();
                    //(*self->selection)  =  ;
                    (*self->marker_selection)  = string(intersected_object->id().c_str());
                    self->is_left_in_motion =  false;
                    self->is_hand_in_motion =  false;
                  }
                }
                else {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  //(*self->selection)  =  ;
                  (*self->marker_selection)  = string(intersected_object->id().c_str());
                  self->is_left_in_motion =  false;
                  self->is_hand_in_motion =  false;
                 }
            }
            else 
            {
              // clear previous selections
              string no_selection = " ";
              self->robotPlanListener->_gl_right_foot->highlight_link(no_selection); 
            }  // end if-else intersected_object !=NULL;
                       
        }// end if(self->robotPlanListener->_gl_right_foot->is_bodypose_adjustment_enabled())        
      } //end ((..._left_hand)&&(..._right_hand)&&(..._left_foot)&&(..._right_foot)) 

     if((self->robotPlanListener->_gl_robot_keyframe_list.size()>2)
      &&(!self->robotPlanListener->_gl_left_hand->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_right_hand->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_left_foot->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_right_foot->is_bodypose_adjustment_enabled())) // knot points other than end points exist
     {
        //for(uint i = 1; i < self->robotPlanListener->_gl_robot_keyframe_list.size()-1; i++) // ignore current state and end state, only intermediate stuff
        if(self->adjust_endstate)
        {
          for(uint i = 1; i < self->robotPlanListener->_gl_robot_keyframe_list.size(); i++) // ignore current state, only intermediate stuff and end state
          { 
            self->robotPlanListener->_gl_robot_keyframe_list[i]->_collision_detector->ray_test( from, to, intersected_object,hit_pt );
            if( intersected_object != NULL ){
              Eigen::Vector3f diff = (from-hit_pt);
              double distance = diff.norm();
              if(shortest_distance>0) {
                if (distance < shortest_distance)
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  self->selected_keyframe_index=i;
                  (*self->marker_selection)  = " ";
              }
              else {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                self->selected_keyframe_index=i;
                (*self->marker_selection)  = " ";
               }
              intersected_object = NULL; 
            }
            
          }//end for  
        }
        else{
          for(uint i = 1; i < self->robotPlanListener->_gl_robot_keyframe_list.size()-1; i++) // ignore current state, only intermediate stuff and end state
            { 
              self->robotPlanListener->_gl_robot_keyframe_list[i]->_collision_detector->ray_test( from, to, intersected_object,hit_pt );
              if( intersected_object != NULL ){
                Eigen::Vector3f diff = (from-hit_pt);
                double distance = diff.norm();
                if(shortest_distance>0) {
                  if (distance < shortest_distance)
                    shortest_distance = distance;
                    self->ray_hit = hit_pt;
                    self->ray_hit_drag = hit_pt;
                    self->ray_hit_t = (hit_pt - self->ray_start).norm();
                    self->selected_keyframe_index=i;
                    (*self->marker_selection)  = " ";
                }
                else {
                  shortest_distance = distance;
                  self->ray_hit = hit_pt;
                  self->ray_hit_drag = hit_pt;
                  self->ray_hit_t = (hit_pt - self->ray_start).norm();
                  self->selected_keyframe_index=i;
                  (*self->marker_selection)  = " ";
                 }
                intersected_object = NULL; 
              }
              
            }//end for 
        }
  
     }// end if _gl_robot_keyframe_list.size()>2  
         
    self->prev_ray_hit_drag = self->ray_hit_drag;   
    return shortest_distance;  
  }
// ===================================================================
  inline static double get_shortest_distance_from_a_plan_frame (void *user,Eigen::Vector3f &from,Eigen::Vector3f &to)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    collision::Collision_Object * intersected_object = NULL;
    Eigen::Vector3f hit_pt;
    double shortest_distance = -1;
    
     if((self->displayed_plan_index!=-1)&&(self->robotPlanListener->_gl_robot_list.size()>0)) {
        self->robotPlanListener->_gl_robot_list[self->displayed_plan_index]->_collision_detector->ray_test( from, to, intersected_object,hit_pt );
        if( intersected_object != NULL ){
            Eigen::Vector3f diff = (from-hit_pt);
            //shortest_distance = diff.norm();
            //self->selected_plan_index=self->displayed_plan_index;
            double distance =diff.norm();
            if(shortest_distance>0) {
              if (distance < shortest_distance)
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                self->selected_plan_index=self->displayed_plan_index;
                self->selected_keyframe_index=-1;
            }
            else {
              shortest_distance = distance;
              self->ray_hit = hit_pt;
              self->ray_hit_drag = hit_pt;
              self->ray_hit_t = (hit_pt - self->ray_start).norm();
              self->selected_plan_index=self->displayed_plan_index;
              self->selected_keyframe_index=-1;
            }
       }
      }
    
         
    self->prev_ray_hit_drag = self->ray_hit_drag;   
    return shortest_distance;  
  }
  
// ===================================================================

  inline static void publish_traj_opt_constraint(void *user,string &channel, int index)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;

    drc::traj_opt_constraint_t msg;
    msg.robot_name =  self->robotPlanListener->_robot_name;    

    int64_t utime = self->robot_utime; // usually this should come from the ROBOT_UTIME subscription, which is broadcast from ros2lcm translator via gazebo.
    //utime = self->robotPlanListener->_last_plan_msg_timestamp; // just for debugging use this; 
    msg.utime = utime;
    
    
     //Publish in palm frame; Handle hand to palm transform in the planner.
    KDL::Frame T_worldframe_palm_l,T_worldframe_palm_r,T_worldframe_foot_l,T_worldframe_foot_r;         
    self->robotPlanListener->_gl_left_hand->get_link_frame("left_palm",T_worldframe_palm_l);
    self->robotPlanListener->_gl_right_hand->get_link_frame("right_palm",T_worldframe_palm_r);
    self->robotPlanListener->_gl_left_foot->get_link_frame("l_foot",T_worldframe_foot_l);
    self->robotPlanListener->_gl_right_foot->get_link_frame("r_foot",T_worldframe_foot_r);       
    /*
    //Publish in hand frame; 
    KDL::Frame T_hand_palm_l = KDL::Frame::Identity();
    hand_palm_l.p[1]= 0.1;
    T_hand_palm_l.M = KDL::Rotation::RPY(M_PI/2,0,M_PI/2,);

    KDL::Frame T_hand_palm_r = KDL::Frame::Identity();
    hand_palm_r.p[1]= -0.1;
    T_hand_palm_r.M = KDL::Rotation::RPY(-M_PI/2,0,-M_PI/2,);
  
    KDL::Frame T_worldframe_hand_l = T_worldframe_palm_l*(T_hand_palm_l.Inverse());
    KDL::Frame T_worldframe_hand_r = T_worldframe_palm_r*(T_hand_palm_r.Inverse());*/
      

    drc::position_3d_t pose; 
    int64_t keyframe_timestamp = self->robotPlanListener->get_keyframe_timestamp(self->selected_keyframe_index);
    //cout<<"keyframe_index:" << self->selected_keyframe_index <<" keyframe_timestamp: " <<keyframe_timestamp << endl;
    msg.num_links = 1;
    if(self->is_hand_in_motion){
      if(self->is_left_in_motion) {  
        msg.link_name.push_back("left_palm");  
        transformKDLToLCM(T_worldframe_palm_l,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
      }
      else {
        msg.link_name.push_back("right_palm");
        transformKDLToLCM(T_worldframe_palm_r,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
      }
    }
    else{
      if(self->is_left_in_motion) {  
        msg.link_name.push_back("l_foot");  
        transformKDLToLCM(T_worldframe_foot_l,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
      }
      else {
        msg.link_name.push_back("r_foot");
        transformKDLToLCM(T_worldframe_foot_r,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
      }
    
    }
    
    msg.num_joints = 0;
    self->lcm->publish(channel, &msg);

  }  
  
// ===================================================================  
  inline static void adjust_keyframe_on_marker_motion(void *user)
  {
      RendererRobotPlan *self = (RendererRobotPlan*) user;
      int index = self->selected_keyframe_index;
      /*int index = self->robotPlanListener->get_motion_copy_index();// gets the in motion copy's index, index can changes as plan is updated.  it no motion copy exists it returns -1, this should never happen.
      if(index==-1){
         cerr << "ERROR: adjust_keyframe_on_marker_motion in robot plan renderer called but no in_motion_copy exists"<< endl;
         return;
      } */

      double gain = 1;      
      // set desired state
      KDL::Frame T_world_ee;
      if(self->is_hand_in_motion){
        if(self->is_left_in_motion) {
          T_world_ee = self->robotPlanListener->_gl_left_hand->_T_world_body;
        }
        else{
          T_world_ee = self->robotPlanListener->_gl_right_hand->_T_world_body;
        }
      }
      else{
        if(self->is_left_in_motion) {
          T_world_ee = self->robotPlanListener->_gl_left_foot->_T_world_body;
        }
        else{
          T_world_ee = self->robotPlanListener->_gl_right_foot->_T_world_body;
        }
      
      
      }

       double currentAngle, angleTo,dtheta;       
       KDL::Frame DragRotation=KDL::Frame::Identity();

           //cout << (*self->marker_selection) << endl;
        if((*self->marker_selection)=="markers::base_x"){
          double dx =  self->ray_hit_drag[0]-self->marker_offset_on_press[0]; // marker_offset_on_press is {hit_location - hand_position} offset @ on mouse press.
          T_world_ee.p[0] = dx;
        }
        else if((*self->marker_selection)=="markers::base_y"){
          double dy =  self->ray_hit_drag[1]-self->marker_offset_on_press[1];
          T_world_ee.p[1] = dy;
        }
        else if((*self->marker_selection)=="markers::base_z"){
          double dz =  self->ray_hit_drag[2]-self->marker_offset_on_press[2];
          T_world_ee.p[2] = dz;
        }
        else if((*self->marker_selection)=="markers::base_roll"){
          currentAngle = atan2(self->prev_ray_hit_drag[2]-T_world_ee.p[2],self->prev_ray_hit_drag[1]-T_world_ee.p[1]);
          angleTo = atan2(self->ray_hit_drag[2]-T_world_ee.p[2],self->ray_hit_drag[1]-T_world_ee.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if((*self->marker_selection)=="markers::base_pitch"){ 
          currentAngle = atan2(self->prev_ray_hit_drag[0]-T_world_ee.p[0],self->prev_ray_hit_drag[2]-T_world_ee.p[2]);
          angleTo = atan2(self->ray_hit_drag[0]-T_world_ee.p[0],self->ray_hit_drag[2]-T_world_ee.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        } 
        else if((*self->marker_selection)=="markers::base_yaw"){
          currentAngle = atan2(self->prev_ray_hit_drag[1]-T_world_ee.p[1],self->prev_ray_hit_drag[0]-T_world_ee.p[0]);
          angleTo = atan2(self->ray_hit_drag[1]-T_world_ee.p[1],self->ray_hit_drag[0]-T_world_ee.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        T_world_ee.M  = DragRotation.M*T_world_ee.M;  
        
        std::map<std::string, double> jointpos_in;
        if(self->is_hand_in_motion){
          if(self->is_left_in_motion) {
            jointpos_in = self->robotPlanListener->_gl_left_hand->_current_jointpos;
            self->robotPlanListener->_gl_left_hand->set_state(T_world_ee,jointpos_in); 
          }
          else {
            jointpos_in = self->robotPlanListener->_gl_right_hand->_current_jointpos;
            self->robotPlanListener->_gl_right_hand->set_state(T_world_ee,jointpos_in); 
          }
        }
        else{
          if(self->is_left_in_motion) {
            jointpos_in = self->robotPlanListener->_gl_left_foot->_current_jointpos;
            self->robotPlanListener->_gl_left_foot->set_state(T_world_ee,jointpos_in); 
          }
          else {
            jointpos_in = self->robotPlanListener->_gl_right_foot->_current_jointpos;
            self->robotPlanListener->_gl_right_foot->set_state(T_world_ee,jointpos_in); 
          }
        }
       
        self->prev_ray_hit_drag = self->ray_hit_drag;
//        string channel = "MANIP_PLAN_CONSTRAINT";
//        publish_traj_opt_constraint(self,channel,self->selected_keyframe_index);
  }   // adjust_keyframe_on_marker_motion()
  
  
}//end namespace

void setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm);

#endif //RENDERER_ROBOTPLAN_HPP
