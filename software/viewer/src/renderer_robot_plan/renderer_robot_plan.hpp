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
#include <visualization_utils/keyboard_signal_utils.hpp>
#include <visualization_utils/affordance_utils/aff_trigger_signal_utils.hpp>
#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;
//using namespace renderer_robot_plan;
namespace renderer_robot_plan_gui_utils{
 static void spawn_plan_storage_addition_popup(void* user);
}
namespace renderer_robot_plan 
{

  enum marker_group_type
  {
    HANDS=0,PELVIS,COM,FEET
  };

  typedef struct _RendererRobotPlan
  {
    BotRenderer renderer;
    BotViewer          *viewer;
    BotGtkParamWidget *pw;
    boost::shared_ptr<RobotPlanListener> robotPlanListener;
    boost::shared_ptr<KeyboardSignalHandler> keyboardSignalHndlr;
    boost::shared_ptr<AffTriggerSignalsHandler> affTriggerSignalsHndlr;
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
    bool show_keyframes;
    Eigen::Vector3f ray_start;
    Eigen::Vector3f ray_end;
    Eigen::Vector3f ray_hit;
    Eigen::Vector3f ray_hit_drag;
    Eigen::Vector3f prev_ray_hit_drag;
    Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
    double ray_hit_t;
    
    std::string* selection;
    std::string* marker_selection;
    
    int  marker_choice_state;
    bool is_left_in_motion;
    int  in_motion_state;
    int selected_plan_index;
    int selected_keyframe_index;
    int displayed_plan_index;
    // Our only source of a free running clock:
    int64_t robot_utime;
    GtkWidget *plan_execution_dock;
    GtkWidget *multiapprove_plan_execution_dock;
    GtkWidget *plan_approval_dock;
    GtkWidget *breakpoint_entry;
    GtkWidget *afftriggered_popup;
    std::string* trigger_source_otdf_id;
    KDL::Frame T_world_trigger_aff;
    
    // Vicon seed planning collection settings:
    int vicon_n_plan_samples;
    double vicon_sample_period;
    int8_t vicon_type;
    
    void keyboardSignalCallback(int keyval, bool is_pressed)
    {
    
      int last_marker_choice = FEET; 
      if((!is_pressed)&&(keyval ==  BKSPACE ))
      {
        cout << "RendererRobotPlan:: back-space pressed. If cache available, will undo replan or plan adjustment." << endl;
        this->robotPlanListener-> setManipPlanFromBackUp();
      }
      if(this->robotPlanListener->is_manip_plan())
      {
        if((!is_pressed)&&(keyval ==  LFTARRW))
        {
          if(this->marker_choice_state>0)
            this->marker_choice_state--;
          else
            this->marker_choice_state = last_marker_choice;
          //cout << "Left Arrow " << this->marker_choice_state << endl;
        }
        else if((!is_pressed)&&(keyval ==  RGTARRW))
        {
          if(this->marker_choice_state<last_marker_choice)
            this->marker_choice_state++;
          else
            this->marker_choice_state = 0;
          //cout << "Right Arrow " << this->marker_choice_state << endl;
        }
        else if((!is_pressed)&&(keyval ==  UPARRW))
        {
          if(this->marker_choice_state<last_marker_choice)
            this->marker_choice_state++;
          else
            this->marker_choice_state = 0;
          //cout << "Up Arrow " << this->marker_choice_state << endl;
        }
        else if((!is_pressed)&&(keyval ==  DWNARRW))
        {
          if(this->marker_choice_state>0)
            this->marker_choice_state--;
          else
            this->marker_choice_state = last_marker_choice;
         //cout << "Down Arrow " << this->marker_choice_state << endl;
        }  
      }    
    }
  
    void affTriggerSignalsCallback(aff_trigger_type type,string otdf_id,KDL::Frame T_world_aff,string plan_id)
    {
      if(type==PLAN_STORE){
          cout<< otdf_id << " got triggered to store currently active plan"<< endl;
          (*this->trigger_source_otdf_id) = otdf_id;
          this->T_world_trigger_aff = T_world_aff;
          //cout << "T_world_aff.p: "<< T_world_aff.p[0] << " " << T_world_aff.p[1] <<" "<< T_world_aff.p[2] << endl;
          renderer_robot_plan_gui_utils::spawn_plan_storage_addition_popup(this);
      }
      else if(type==PLAN_LOAD){
        cout<< otdf_id << " got triggered to load stored plan : "<< plan_id << endl;
        (*this->trigger_source_otdf_id) = otdf_id;
          this->T_world_trigger_aff = T_world_aff;
          //cout << "T_world_aff.p: "<< T_world_aff.p[0] << " " << T_world_aff.p[1] <<" "<< T_world_aff.p[2] << endl;
          std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
          std::string otdf_filepath,plan_xml_dirpath;
          otdf_filepath =  otdf_models_path + (*this->trigger_source_otdf_id) +".otdf";
          plan_xml_dirpath =  otdf_models_path + "stored_plans/";
          PlanSeed planSeed;
          planSeed.loadFromOTDF(otdf_filepath,plan_xml_dirpath,plan_id);
          this->robotPlanListener->setPlanFromStorage(this->T_world_trigger_aff,
                                                      planSeed.plan_type,
                                                      planSeed.stateframe_ids,
                                                      planSeed.stateframe_values,
                                                      planSeed.graspframe_ids,
                                                      planSeed.graspframe_values);
      }
      else
          cerr<<  " unknown trigger "<< endl;
    }
    
  } RendererRobotPlan;
  
// ===================================================================
// Distance queries
// ===================================================================

  inline static bool get_shortest_distance_from_marker_object(void *user,Eigen::Vector3f &from,Eigen::Vector3f &to,Eigen::Vector3f &hit_pt, boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_marker, collision::Collision_Object * intersected_object,double& shortest_distance)
  {
    bool value = false;
     RendererRobotPlan *self = (RendererRobotPlan*) user;
     if(_gl_marker->is_bodypose_adjustment_enabled())
      {
          _gl_marker->_collision_detector_floatingbase_markers->ray_test( from, to, intersected_object,hit_pt);
          
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
                   value =  true;
                }
              }
              else {
                shortest_distance = distance;
                self->ray_hit = hit_pt;
                self->ray_hit_drag = hit_pt;
                self->ray_hit_t = (hit_pt - self->ray_start).norm();
                //(*self->selection)  =  ;
                (*self->marker_selection)  = string(intersected_object->id().c_str());
                 value =  true;
               }
          }
          else 
          {
            // clear previous selections
            string no_selection = " ";
            _gl_marker->highlight_link(no_selection); 
          }  // end if-else intersected_object !=NULL;
                     
      }// end if(_gl_marker->is_bodypose_adjustment_enabled())
      
      return value;
  } // end get get_shortest_distance_from_marker_object


  inline static double get_shortest_distance_between_keyframes_and_markers (void *user,Eigen::Vector3f &from,Eigen::Vector3f &to)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    collision::Collision_Object * intersected_object = NULL;
    Eigen::Vector3f hit_pt;
    double shortest_distance =-1; 
   // self->selected_keyframe_index=-1; Do not clear this on marker selection, selected markers need to know which keyframe index they are associated with
  
       if((self->robotPlanListener->_gl_left_hand)&&(self->robotPlanListener->_gl_right_hand)
        &&(self->robotPlanListener->_gl_left_foot)&&(self->robotPlanListener->_gl_right_foot)
        &&(self->robotPlanListener->_gl_pelvis)&&(self->robotPlanListener->_gl_com)) // exists
      {
        //  Marker Foviation Logic 
        if(self->marker_choice_state == HANDS)
        {
          if(get_shortest_distance_from_marker_object(self,from,to,hit_pt,self->robotPlanListener->_gl_left_hand,intersected_object,shortest_distance))
          {
            self->is_left_in_motion = true;
            self->in_motion_state = HANDS;
          }
          if(get_shortest_distance_from_marker_object(self,from,to,hit_pt,self->robotPlanListener->_gl_right_hand,intersected_object,shortest_distance))
          {
            self->is_left_in_motion = false;
            self->in_motion_state = HANDS;
          }
        }
        else if(self->marker_choice_state == FEET)
        {
          if(get_shortest_distance_from_marker_object(self,from,to,hit_pt,self->robotPlanListener->_gl_left_foot,intersected_object,shortest_distance))
          {
            self->is_left_in_motion = true;
            self->in_motion_state = FEET;
          }
          if(get_shortest_distance_from_marker_object(self,from,to,hit_pt,self->robotPlanListener->_gl_right_foot,intersected_object,shortest_distance))
          {
            self->is_left_in_motion = false;
            self->in_motion_state = FEET;
          } 
        }  
        else if(self->marker_choice_state == PELVIS)
        {
          if(get_shortest_distance_from_marker_object(self,from,to,hit_pt,self->robotPlanListener->_gl_pelvis,intersected_object,shortest_distance))
          {
            self->in_motion_state = PELVIS;
          } 
        }
        else if(self->marker_choice_state == COM)
        {
          if(get_shortest_distance_from_marker_object(self,from,to,hit_pt,self->robotPlanListener->_gl_com,intersected_object,shortest_distance))
          {
            self->in_motion_state = COM;
          }      
        }  
         
      } //end ((..._left_hand)&&(..._right_hand)&&(..._left_foot)&&(..._right_foot)&&(..._pelvis)&&(..._com)) 

     if((self->robotPlanListener->_gl_robot_keyframe_list.size()>1)
      &&(!self->robotPlanListener->_gl_left_hand->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_right_hand->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_left_foot->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_right_foot->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_pelvis->is_bodypose_adjustment_enabled())
      &&(!self->robotPlanListener->_gl_com->is_bodypose_adjustment_enabled())) // knot points other than end points exist
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
     if((self->displayed_plan_index!=-1)
         &&(self->robotPlanListener->_gl_robot_list.size()>0)
         &&(self->displayed_plan_index < self->robotPlanListener->_gl_robot_list.size())
        ) 
     {
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
     
    self->robotPlanListener->_gl_left_hand->get_link_frame(self->robotPlanListener->_lhand_ee_name,T_worldframe_palm_l);
    self->robotPlanListener->_gl_right_hand->get_link_frame(self->robotPlanListener->_rhand_ee_name,T_worldframe_palm_r);
    self->robotPlanListener->_gl_left_foot->get_link_frame("l_foot",T_worldframe_foot_l);
    self->robotPlanListener->_gl_right_foot->get_link_frame("r_foot",T_worldframe_foot_r);     

    KDL::Frame T_worldframe_pelvis,T_worldframe_com;      
    self->robotPlanListener->_gl_pelvis->get_link_frame("pelvis",T_worldframe_pelvis);
    self->robotPlanListener->_gl_com->get_link_frame("com_link",T_worldframe_com);  
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
    if(self->in_motion_state == HANDS) {
      if(self->is_left_in_motion) { 
        msg.link_name.push_back(self->robotPlanListener->_lhand_ee_name);  
        transformKDLToLCM(T_worldframe_palm_l,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
      }
      else {
        msg.link_name.push_back(self->robotPlanListener->_rhand_ee_name);
        transformKDLToLCM(T_worldframe_palm_r,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
      }
    }
    else if(self->in_motion_state == FEET) {
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
    else if(self->in_motion_state == PELVIS) {
        msg.link_name.push_back("pelvis");  
        transformKDLToLCM(T_worldframe_pelvis,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
    }
    else if(self->in_motion_state == COM) {
        msg.link_name.push_back("com::");  
        transformKDLToLCM(T_worldframe_com,pose); 
        msg.link_origin_position.push_back(pose);
        msg.link_timestamps.push_back(keyframe_timestamp);
    }    
    msg.num_joints = 0;
    self->lcm->publish(channel, &msg);

  }  
  
// ===================================================================  
  inline static void adjust_keyframe_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererRobotPlan *self = (RendererRobotPlan*) user;
      int index = self->selected_keyframe_index;
      /*int index = self->robotPlanListener->get_motion_copy_index();// gets the in motion copy's index, index can changes as plan is updated.  it no motion copy exists it returns -1, this should never happen.*/
      if(index==-1){
         cerr << "ERROR: adjust_keyframe_on_marker_motion in robot plan renderer called but no keyframe exists"<< endl;
         return;
      } 

      double gain = 1;      
      // set desired state
      string root_link_name;
      KDL::Frame T_world_ee;
      std::string token  = "plane::";
      size_t found = (*self->marker_selection).find(token);  
      string plane_name="";
       KDL::Frame T_marker_world;
      
      // Marker Foviation Logic
      if(self->in_motion_state == HANDS) {
        if(self->is_left_in_motion) {
          T_world_ee = self->robotPlanListener->_gl_left_hand->_T_world_body;
          T_marker_world  =  (self->robotPlanListener->_gl_left_hand->get_marker_frame()).Inverse(); 
          root_link_name=self->robotPlanListener->_gl_left_hand->get_root_link_name();
          if(found!=std::string::npos) 
              self->robotPlanListener->_gl_left_hand->extract_plane_name(root_link_name,plane_name);
        }
        else{
          T_world_ee = self->robotPlanListener->_gl_right_hand->_T_world_body;
          T_marker_world  =  (self->robotPlanListener->_gl_right_hand->get_marker_frame()).Inverse(); 
          root_link_name=self->robotPlanListener->_gl_right_hand->get_root_link_name();
          if(found!=std::string::npos) 
              self->robotPlanListener->_gl_right_hand->extract_plane_name(root_link_name,plane_name);
        }
        
      }
      else if(self->in_motion_state == FEET) {
        if(self->is_left_in_motion) {
          T_world_ee = self->robotPlanListener->_gl_left_foot->_T_world_body;
          T_marker_world  =  (self->robotPlanListener->_gl_left_foot->get_marker_frame()).Inverse(); 
          root_link_name=self->robotPlanListener->_gl_left_foot->get_root_link_name();
          if(found!=std::string::npos) 
              self->robotPlanListener->_gl_left_foot->extract_plane_name(root_link_name,plane_name);
        }
        else{
          T_world_ee = self->robotPlanListener->_gl_right_foot->_T_world_body;
          T_marker_world  =  (self->robotPlanListener->_gl_right_foot->get_marker_frame()).Inverse(); 
          root_link_name=self->robotPlanListener->_gl_right_foot->get_root_link_name();
          if(found!=std::string::npos) 
              self->robotPlanListener->_gl_right_foot->extract_plane_name(root_link_name,plane_name);
        }
      
      }
      else if(self->in_motion_state == PELVIS) {
        T_world_ee = self->robotPlanListener->_gl_pelvis->_T_world_body;
        T_marker_world  =  (self->robotPlanListener->_gl_pelvis->get_marker_frame()).Inverse(); 
        root_link_name=self->robotPlanListener->_gl_pelvis->get_root_link_name();
        if(found!=std::string::npos) 
            self->robotPlanListener->_gl_pelvis->extract_plane_name(root_link_name,plane_name);
      }
      else if(self->in_motion_state == COM) {
        T_world_ee = self->robotPlanListener->_gl_com->_T_world_body;
        T_marker_world  =  (self->robotPlanListener->_gl_com->get_marker_frame()).Inverse(); 
        root_link_name=self->robotPlanListener->_gl_com->get_root_link_name();
        if(found!=std::string::npos) 
            self->robotPlanListener->_gl_com->extract_plane_name(root_link_name,plane_name);
      }      
      
       double currentAngle, angleTo,dtheta;       
       KDL::Frame DragRotation=KDL::Frame::Identity();
       
      KDL::Frame T_marker_ee = T_marker_world*T_world_ee;
      Eigen::Vector3f markerframe_prev_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_marker_world,markerframe_prev_ray_hit_drag);
      Eigen::Vector3f markerframe_ray_hit_drag;
      rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_marker_world,markerframe_ray_hit_drag);
      Eigen::Vector3f worldframe_delta,markerframe_delta;
      worldframe_delta  = self->ray_hit_drag-self->marker_offset_on_press;
      rotate_eigen_vector_given_kdl_frame(worldframe_delta,T_marker_world,markerframe_delta);          

        if(found!=std::string::npos)  
        {
          size_t found2 = plane_name.find("x"); 
          bool x_plane_active = (found2!=std::string::npos);
          found2 = plane_name.find("y"); 
          bool y_plane_active = (found2!=std::string::npos);       
          found2 = plane_name.find("z"); 
          bool z_plane_active = (found2!=std::string::npos);
          if(x_plane_active){
           T_marker_ee.p[0] = markerframe_delta[0];
          }
          if(y_plane_active){
            T_marker_ee.p[1] =markerframe_delta[1];
          }
          if(z_plane_active){
            T_marker_ee.p[2] =markerframe_delta[2];
          }        
        }  

           //cout << (*self->marker_selection) << endl;
        if((*self->marker_selection)=="markers::base_x"){
          double dx =  self->ray_hit_drag[0]-self->marker_offset_on_press[0]; // marker_offset_on_press is {hit_location - hand_position} offset @ on mouse press.
          T_marker_ee.p[0] = markerframe_delta[0];
        }
        else if((*self->marker_selection)=="markers::base_y"){
          double dy =  self->ray_hit_drag[1]-self->marker_offset_on_press[1];
          T_marker_ee.p[1] = markerframe_delta[1];
        }
        else if((*self->marker_selection)=="markers::base_z"){
          double dz =  self->ray_hit_drag[2]-self->marker_offset_on_press[2];
          T_marker_ee.p[2] = markerframe_delta[2];
        }
        else if((*self->marker_selection)=="markers::base_roll"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[2]-T_marker_ee.p[2],markerframe_prev_ray_hit_drag[1]-T_marker_ee.p[1]);
          angleTo = atan2(markerframe_ray_hit_drag[2]-T_marker_ee.p[2],markerframe_ray_hit_drag[1]-T_marker_ee.p[1]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 1; axis[1] = 0; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        else if((*self->marker_selection)=="markers::base_pitch"){ 
          currentAngle = atan2(markerframe_prev_ray_hit_drag[0]-T_marker_ee.p[0],markerframe_prev_ray_hit_drag[2]-T_marker_ee.p[2]);
          angleTo = atan2(markerframe_ray_hit_drag[0]-T_marker_ee.p[0],markerframe_ray_hit_drag[2]-T_marker_ee.p[2]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          //dtheta =  atan2(sin(angleTo - currentAngle), cos(angleTo - currentAngle));
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 1; axis[2]=0;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        } 
        else if((*self->marker_selection)=="markers::base_yaw"){
          currentAngle = atan2(markerframe_prev_ray_hit_drag[1]-T_marker_ee.p[1],markerframe_prev_ray_hit_drag[0]-T_marker_ee.p[0]);
          angleTo = atan2(markerframe_ray_hit_drag[1]-T_marker_ee.p[1],markerframe_ray_hit_drag[0]-T_marker_ee.p[0]);
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          KDL::Vector axis;
          axis[0] = 0; axis[1] = 0; axis[2]=1;
          DragRotation.M = KDL::Rotation::Rot(axis,dtheta);
        }
        T_marker_ee.M  = DragRotation.M*T_marker_ee.M;  
        T_world_ee = (T_marker_world.Inverse())*T_marker_ee;
        
        std::map<std::string, double> jointpos_in;
        if(self->in_motion_state == HANDS) {
          if(self->is_left_in_motion) {
            jointpos_in = self->robotPlanListener->_gl_left_hand->_current_jointpos;
            self->robotPlanListener->_gl_left_hand->set_state(T_world_ee,jointpos_in); 
          }
          else {
            jointpos_in = self->robotPlanListener->_gl_right_hand->_current_jointpos;
            self->robotPlanListener->_gl_right_hand->set_state(T_world_ee,jointpos_in); 
          }
        }
        else if(self->in_motion_state == FEET) {
          if(self->is_left_in_motion) {
            jointpos_in = self->robotPlanListener->_gl_left_foot->_current_jointpos;
            self->robotPlanListener->_gl_left_foot->set_state(T_world_ee,jointpos_in); 
          }
          else {
            jointpos_in = self->robotPlanListener->_gl_right_foot->_current_jointpos;
            self->robotPlanListener->_gl_right_foot->set_state(T_world_ee,jointpos_in); 
          }
        }
        else if(self->in_motion_state == PELVIS) {
         jointpos_in = self->robotPlanListener->_gl_pelvis->_current_jointpos;
         self->robotPlanListener->_gl_pelvis->set_state(T_world_ee,jointpos_in); 
        }
        else if(self->in_motion_state == COM) {
         jointpos_in = self->robotPlanListener->_gl_com->_current_jointpos;
         self->robotPlanListener->_gl_com->set_state(T_world_ee,jointpos_in); 
        }        
       
        self->prev_ray_hit_drag = self->ray_hit_drag;
//        string channel = "MANIP_PLAN_CONSTRAINT";
//        publish_traj_opt_constraint(self,channel,self->selected_keyframe_index);
  }   // adjust_keyframe_on_marker_motion()
  
  
}//end namespace


// 0 = typical mode, 1 = robot_plan 2 = robot_plan_compressed
void setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm, int operation_mode, KeyboardSignalRef signalRef,AffTriggerSignalsRef affTriggerSignalsRef);
#endif //RENDERER_ROBOTPLAN_HPP
