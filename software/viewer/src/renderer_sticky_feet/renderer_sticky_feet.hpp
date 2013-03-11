#ifndef RENDERER_STICKYFEET_HPP
#define RENDERER_STICKYFEET_HPP

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include "FootStepPlanListener.hpp"
#include <visualization_utils/angles.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;

namespace renderer_sticky_feet 
{


  typedef struct _RendererStickyFeet
  {
    BotRenderer renderer;
    BotViewer *viewer;
    BotGtkParamWidget *pw;
    boost::shared_ptr<FootStepPlanListener> footStepPlanListener;
    boost::shared_ptr<lcm::LCM> lcm;
    int64_t max_draw_utime;
    BotEventHandler ehandler;
    bool ht_auto_adjust_enabled;
    bool clicked;
    bool dragging;

    Eigen::Vector3f ray_start;
    Eigen::Vector3f ray_end;
    Eigen::Vector3f ray_hit;
    Eigen::Vector3f ray_hit_drag;
    Eigen::Vector3f prev_ray_hit_drag;
    Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
    double ray_hit_t;
    
    
    std::string* selection;
    std::string* marker_selection;
    int selected_planned_footstep_index;

    // Our only source of a free running clock:
    int64_t robot_utime;

  } RendererStickyFeet;



  inline static double get_shortest_distance_between_stickyfeet_and_markers (void *user,Eigen::Vector3f &from,Eigen::Vector3f &to)
  {
    RendererStickyFeet *self = (RendererStickyFeet*) user;
    collision::Collision_Object * intersected_object = NULL;
    Eigen::Vector3f hit_pt;
    double shortest_distance = -1;
    
 
    for(uint i = 0; i < self->footStepPlanListener->_gl_planned_stickyfeet_list.size(); i++) 
    { 
    
        if(self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->is_bodypose_adjustment_enabled())
        {

            self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
          
          if(intersected_object != NULL ){
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
                  self->selected_planned_footstep_index = i;
                  (*self->marker_selection)  = string(intersected_object->id().c_str());
                }
              }
              else {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                //(*self->selection)  =  ;
                self->selected_planned_footstep_index = i;
                (*self->marker_selection)  = string(intersected_object->id().c_str());
               }
          }
          else {
          // clear previous selections
           string no_selection = " ";
           self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_link(no_selection); 
          }  
                       
        }  
    

      self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_collision_detector->ray_test( from, to, intersected_object,hit_pt );

      if( intersected_object != NULL ){
        Eigen::Vector3f diff = (from-hit_pt);
        double distance = diff.norm();
        if(shortest_distance>0) {
          if (distance < shortest_distance)
          {
            shortest_distance = distance;
            self->ray_hit = hit_pt;
            self->ray_hit_drag = hit_pt;
            self->ray_hit_t = (hit_pt - self->ray_start).norm();
            self->selected_planned_footstep_index = i;
            (*self->marker_selection)  = " ";
          }
        }
        else {
          shortest_distance = distance;
          self->ray_hit = hit_pt;
          self->ray_hit_drag = hit_pt;
          self->ray_hit_t = (hit_pt - self->ray_start).norm();
          self->selected_planned_footstep_index=i;
          (*self->marker_selection)  = " ";
        }
        intersected_object = NULL; 
        }
        else {
        // clear previous selections
         string no_selection = " ";
         self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_link(no_selection); 
        }  
      
    }//end for      
    
   self->prev_ray_hit_drag = self->ray_hit_drag;                   
   
    return shortest_distance;  
  }


// ===================================================================

  
  inline static void publish_traj_constraint(void *user, uint i, string &channel)
  {
      RendererStickyFeet *self = (RendererStickyFeet*) user;
     KDL::Frame T_world_object = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body;
     
     drc::traj_opt_constraint_t msg;
     msg.utime = self->robot_utime;
     msg.robot_name =  self->footStepPlanListener->_robot_name;
     
     msg.num_links = 1;
     if(self->footStepPlanListener->_planned_stickyfeet_info_list[i]== FootStepPlanListener::LEFT)
      msg.link_name.push_back(self->footStepPlanListener->_left_foot_name);
     else if(self->footStepPlanListener->_planned_stickyfeet_info_list[i]== FootStepPlanListener::RIGHT)
      msg.link_name.push_back(self->footStepPlanListener->_right_foot_name);
     
     drc::position_3d_t pose;
     transformKDLToLCM(T_world_object,pose);
     msg.link_origin_position.push_back(pose);
     msg.link_timestamps.push_back(0.0);// where should this information come from?
     
     msg.num_joints = 0;
     self->lcm->publish(channel, &msg);
  }

// ===================================================================
  inline static void set_object_desired_state_on_marker_motion(void *user)
  {
      RendererStickyFeet *self = (RendererStickyFeet*) user;
      
      double gain = 1;

       uint i = self->selected_planned_footstep_index;
      // set desired state
      KDL::Frame T_world_object = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body;
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->is_bodypose_adjustment_enabled())
      {
        //cout << (*self->marker_selection) << endl;
        if((*self->marker_selection)=="markers::base_x"){
          double dx =  self->ray_hit_drag[0]-self->marker_offset_on_press[0];
          T_world_object.p[0] = dx;
        }
        else if((*self->marker_selection)=="markers::base_y"){
          double dy =  self->ray_hit_drag[1]-self->marker_offset_on_press[1];
          T_world_object.p[1] = dy;
        }      
        else if((*self->marker_selection)=="markers::base_z"){
          double dz =  self->ray_hit_drag[2]-self->marker_offset_on_press[2];
          T_world_object.p[2] = dz;
        }    
        else if((*self->marker_selection)=="markers::base_roll"){
          currentAngle = atan2(self->prev_ray_hit_drag[2]-T_world_object.p[2],self->prev_ray_hit_drag[1]-T_world_object.p[1]);
          angleTo = atan2(self->ray_hit_drag[2]-T_world_object.p[2],self->ray_hit_drag[1]-T_world_object.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if((*self->marker_selection)=="markers::base_pitch"){ 
          currentAngle = atan2(self->prev_ray_hit_drag[0]-T_world_object.p[0],self->prev_ray_hit_drag[2]-T_world_object.p[2]);
          angleTo = atan2(self->ray_hit_drag[0]-T_world_object.p[0],self->ray_hit_drag[2]-T_world_object.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }    
        else if((*self->marker_selection)=="markers::base_yaw"){
          currentAngle = atan2(self->prev_ray_hit_drag[1]-T_world_object.p[1],self->prev_ray_hit_drag[0]-T_world_object.p[0]);
          angleTo = atan2(self->ray_hit_drag[1]-T_world_object.p[1],self->ray_hit_drag[0]-T_world_object.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        
        T_world_object.M  = DragRotation.M*T_world_object.M;  
        
        std::map<std::string, double> jointpos_in;
        jointpos_in = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_current_jointpos;
        self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->set_state(T_world_object,jointpos_in); 
        
        self->prev_ray_hit_drag = self->ray_hit_drag;
        
        string channel = "TRAJ_OPT_CONSTRAINT";
        publish_traj_constraint(self,i,channel);
      
      }
     
  }   // end set_object_desired_state_on_marker_motion()  

  
  

}


void setup_renderer_sticky_feet(BotViewer *viewer, int render_priority, lcm_t *lcm);

#endif //RENDERER_STICKYFEET_HPP
