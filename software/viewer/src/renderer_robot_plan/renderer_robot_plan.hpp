#ifndef RENDERER_ROBOTPLAN_HPP
#define RENDERER_ROBOTPLAN_HPP

#include <iostream>
#include <boost/function.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/atlas_status_t.hpp"
#include "lcmtypes/drc/traj_opt_constraint_t.hpp"
#include "lcmtypes/drc/utime_t.hpp"

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
#include <visualization_utils/foviation_signal_utils.hpp>
#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;

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
    RendererFoviationSignalRef _rendererFoviationSignalRef;
    bool _renderer_foviate;
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
    int atlas_state;
    int64_t atlas_status_utime; 
  
    GtkWidget *plan_execution_dock;
    bool ignore_plan_execution_warning;
    GtkWidget *plan_execute_button;
    GtkWidget *multiapprove_plan_execution_dock;
    GtkWidget *plan_approval_dock;
    GtkWidget *breakpoint_entry;
    GtkWidget *afftriggered_popup;
    std::string* trigger_source_otdf_id;
    KDL::Frame T_world_trigger_aff;
    
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
        else if((!is_pressed)&&(keyval == GDK_KEY_h))
        {
          this->setPlanVisibility(!this->getPlanVisibility());
        }
      }    
    }

    void setPlanVisibility(const bool vis);
    bool getPlanVisibility() const;
  
    void affTriggerSignalsCallback(aff_trigger_type type,string otdf_id,KDL::Frame T_world_aff,string plan_id)
    {
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
    double shortest_distance =-1; 
  
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

  }  
  
// ===================================================================  
  inline static void adjust_keyframe_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RendererRobotPlan *self = (RendererRobotPlan*) user;

  }   // adjust_keyframe_on_marker_motion()
  
  
}//end namespace


// 0 = typical mode, 1 = robot_plan 2 = robot_plan_compressed
void setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm, int operation_mode, KeyboardSignalRef signalRef,AffTriggerSignalsRef affTriggerSignalsRef,RendererFoviationSignalRef rendererFoviationSignalRef);
#endif //RENDERER_ROBOTPLAN_HPP
