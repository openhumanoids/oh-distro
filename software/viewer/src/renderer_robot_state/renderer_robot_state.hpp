#ifndef RENDERER_ROBOT_STATE_HPP
#define RENDERER_ROBOT_STATE_HPP

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <iostream>
#include <boost/function.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/drill_control_t.hpp"
#include "lcmtypes/drc/joint_angles_t.hpp"
#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/traj_opt_constraint_t.hpp"
#include "lcmtypes/drc/walking_goal_t.hpp"
#include <bot_core/rotations.h>
#include <Eigen/Dense>
#include <visualization_utils/angles.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/keyboard_signal_utils.hpp>
#include <visualization_utils/affordance_utils/aff_trigger_signal_utils.hpp>
#include <visualization_utils/foviation_signal_utils.hpp>
#include "RobotStateListener.hpp"

#define PARAM_SELECTION "Enable Selection"
#define PARAM_WIRE "Show BBoxs For Meshes"  
#define PARAM_COLOR_ALPHA "Alpha"
#define PARAM_CURRENT_JOINTPOS "Current joint pos"
#define PARAM_ENABLE_POSTURE_ADJUSTMENT "Set Desired Posture"
#define PARAM_SEND_POSTURE_GOAL_BACK_ZEROED "Zero Selected Back Joints"
#define PARAM_RESET_POSTURE "Reset"
#define PARAM_SHOW_FORCES "Show EE Forces (0.05*(f_meas/g))"
#define PARAM_ENABLE_EE_TELEOP "FineGrained EE Teleop"
#define PARAM_BACK_ROLL "Roll"
#define PARAM_BACK_PITCH "Pitch"
#define PARAM_BACK_YAW "Yaw"
#define PARAM_BACK_FIX_HANDS "Fix Hands"

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;

typedef struct _ZeroBackOptions {
  bool roll;
  bool pitch;
  bool yaw;
  bool fix_hands;
} ZeroBackOptions;

namespace renderer_robot_state_gui_utils{
 static void spawn_endpose_storage_addition_popup(void* user);
}
namespace renderer_robot_state 
{
  static void publish_candidate_sticky_feet(void *user, const string& channel,string &otdf_uid, KDL::Frame &T_world_aff, bool desired_state);
   
  typedef struct _RobotStateRendererStruc 
  {
    BotRenderer renderer;
    BotViewer          *viewer;
    BotGtkParamWidget *pw;
    boost::shared_ptr<renderer_robot_state::RobotStateListener> robotStateListener;
    boost::shared_ptr<lcm::LCM> lcm;
    boost::shared_ptr<KeyboardSignalHandler> keyboardSignalHndlr;
    boost::shared_ptr<AffTriggerSignalsHandler> affTriggerSignalsHndlr;
    RendererFoviationSignalRef _rendererFoviationSignalRef;
    bool _renderer_foviate;
    //BotEventHandler *key_handler;
    BotEventHandler ehandler;
    bool selection_enabled;
    bool clicked;
    bool dragging; 
    bool visualize_bbox;
    bool visualize_forces;
    
    GtkWidget *pose_approval_dock;
    GtkWidget *teleop_popup;
    GtkWidget *teleop_error_entry;
    double active_res;
    int active_ee;
    double active_angres;

    std::string* selection;
    std::string* marker_selection;
    
    
    Eigen::Vector3f ray_start;
    Eigen::Vector3f ray_end;
    Eigen::Vector3f ray_hit;
    Eigen::Vector3f ray_hit_drag;
    Eigen::Vector3f prev_ray_hit_drag;
    Eigen::Vector3f ray_hit_normal;
    Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
    double ray_hit_t;

    // transparency of the model:
    float alpha;
    GtkWidget *afftriggered_popup;
    std::string* trigger_source_otdf_id;
    KDL::Frame T_world_trigger_aff;

    ZeroBackOptions zero_back_options;
    
     void keyboardSignalCallback(int keyval, bool is_pressed)
    {
      //
    }
  
    void affTriggerSignalsCallback(aff_trigger_type type,string otdf_uid,KDL::Frame T_world_aff,string pose_id)
    {
      if(type==CURRENT_FOOTSTEPS_REQUEST){
          cout<< otdf_uid << " aff renderer is requesting current robot state footsteps to generate and store sticky feet"<< endl;
          //(*this->trigger_source_otdf_id) = otdf_uid;
          //this->T_world_trigger_aff = T_world_aff;
           //cout << "T_world_aff.p: "<< T_world_aff.p[0] << " " << T_world_aff.p[1] <<" "<< T_world_aff.p[2] << endl;
          string channel = "AFF_TRIGGERED_CANDIDATE_STICKY_FEET";
          publish_candidate_sticky_feet(this,channel,otdf_uid,T_world_aff,false);
      }
      else if(type==DESIRED_FOOTSTEPS_REQUEST){
        cout<< otdf_uid << " aff renderer is requesting desired robot state footsteps  to generate and store sticky feet: "<< endl;
        //(*this->trigger_source_otdf_id) = otdf_uid;
         // this->T_world_trigger_aff = T_world_aff;
          //cout << "T_world_aff.p: "<< T_world_aff.p[0] << " " << T_world_aff.p[1] <<" "<< T_world_aff.p[2] << endl;
          //std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
          //std::string otdf_filepath,plan_xml_dirpath;
          //otdf_filepath =  otdf_models_path + (*this->trigger_source_otdf_id) +".otdf";
          string channel = "AFF_TRIGGERED_CANDIDATE_STICKY_FEET";
          publish_candidate_sticky_feet(this,channel,otdf_uid,T_world_aff,true);          
      }
      else if(type==POSE_STORE){
          cout<< otdf_uid << " got triggered to store currently active pose"<< endl;
          (*this->trigger_source_otdf_id) = otdf_uid;
          this->T_world_trigger_aff = T_world_aff;
          //cout << "T_world_aff.p: "<< T_world_aff.p[0] << " " << T_world_aff.p[1] <<" "<< T_world_aff.p[2] << endl;
          renderer_robot_state_gui_utils::spawn_endpose_storage_addition_popup(this);
      }
      else if(type==POSE_LOAD){
        cout<< otdf_uid << " got triggered to load stored endpose : "<< pose_id << endl;
        (*this->trigger_source_otdf_id) = otdf_uid;
          this->T_world_trigger_aff = T_world_aff;
          //cout << "T_world_aff.p: "<< T_world_aff.p[0] << " " << T_world_aff.p[1] <<" "<< T_world_aff.p[2] << endl;
          std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
          std::string otdf_filepath,pose_xml_dirpath;
          otdf_filepath =  otdf_models_path + (*this->trigger_source_otdf_id) +".otdf";
          pose_xml_dirpath =  otdf_models_path + "stored_poses/";
          PoseSeed poseSeed;
          poseSeed.loadFromOTDF(otdf_filepath,pose_xml_dirpath,pose_id);
          this->robotStateListener->setDesiredStateFromStorage(this->T_world_trigger_aff,
                                                               poseSeed.stateframe_ids,
                                                               poseSeed.stateframe_values);
      }
    }
    
    
    void handleCannedPostureGoalMsg(const lcm::ReceiveBuffer* rbuf,
	          const std::string& chan, 
	          const drc::joint_angles_t* msg)
    {
       cout << "received canned posture goal" << endl;
       if(!this->robotStateListener->_gl_robot->is_future_state_changing())
         this->robotStateListener->_gl_robot->set_future_state_changing(true);
         
        std::map<std::string, double> jointpos_in;
        jointpos_in = this->robotStateListener->_gl_robot->_current_jointpos;
        for(size_t j=0;j<msg->num_joints;j++)
        {
           jointpos_in.find(msg->joint_name[j])->second = msg->joint_position[j];
        }
        this->robotStateListener->_gl_robot->set_future_state(this->robotStateListener->_gl_robot->_T_world_body,jointpos_in); 
    }
    
     void handleCommittedOrRejectedRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
	          const std::string& chan, 
	          const drc::robot_plan_t* msg)
    {
       // reset future robot state on plan commit
       if(this->robotStateListener->_gl_robot->is_future_state_changing())
       this->robotStateListener->_gl_robot->set_future_state_changing(false);
       this->robotStateListener->_gl_robot->set_future_state( this->robotStateListener->_gl_robot->_T_world_body, this->robotStateListener->_gl_robot->_current_jointpos);   
       this->robotStateListener->_gl_robot->disable_future_display();      
       bot_viewer_request_redraw(this->viewer); 
    }   
    
    
  } RobotStateRendererStruc;

inline static double get_shortest_distance_between_robot_links_and_jointdof_markers (void *user,Eigen::Vector3f &from,Eigen::Vector3f &to)
{
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    collision::Collision_Object * intersected_object = NULL;
    Eigen::Vector3f hit_pt;
    double shortest_distance = -1;  

    if(self->robotStateListener->_gl_robot) // to make sure that _gl_robot is initialized 
    {
     //self->robotStateListener->_gl_robot->_collision_detector->num_collisions();
       if(self->robotStateListener->_gl_robot->is_jointdof_adjustment_enabled())
          self->robotStateListener->_gl_robot->_collision_detector_jointdof_markers->ray_test( from, to, intersected_object,hit_pt);
       else
          self->robotStateListener->_gl_robot->_collision_detector->ray_test(from, to, intersected_object,hit_pt);
    }
    if( intersected_object != NULL ){
      Eigen::Vector3f diff = (from-hit_pt);
        double distance = diff.norm();
       // std::cout  << "RobotStateRenderer distance " << distance << std::endl;
     
       if(shortest_distance>0) {
          shortest_distance = distance;
          self->ray_hit = hit_pt;
          self->ray_hit_drag = hit_pt;
          self->ray_hit_t = (hit_pt - self->ray_start).norm();
          if(self->robotStateListener->_gl_robot->is_jointdof_adjustment_enabled())
           (*self->marker_selection)  = string(intersected_object->id().c_str());
          else
           (*self->selection)  = std::string(intersected_object->id().c_str());       
       }
       else {
          shortest_distance = distance;
          self->ray_hit = hit_pt;
          self->ray_hit_drag = hit_pt;
          self->ray_hit_t = (hit_pt - self->ray_start).norm();
          if(self->robotStateListener->_gl_robot->is_jointdof_adjustment_enabled())
           (*self->marker_selection)  = string(intersected_object->id().c_str());
          else
           (*self->selection)  = std::string(intersected_object->id().c_str());
       }
    }
    else {
       (*self->selection)  = " ";
       (*self->marker_selection)  = " ";
       string no_selection = " ";
       self->robotStateListener->_gl_robot->highlight_link(no_selection); 
       self->robotStateListener->_gl_robot->highlight_marker(no_selection);
    }
    
    
    self->prev_ray_hit_drag = self->ray_hit_drag;   
    return shortest_distance;  
}





 inline static void set_desired_robot_posture_on_marker_motion(void *user,Eigen::Vector3f start,Eigen::Vector3f dir)
  {
      RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
      double gain = 1;

      if(!self->robotStateListener->_gl_robot->is_future_state_changing()) {
        self->robotStateListener->_gl_robot->set_future_state_changing(true);  
       }//end if(!self->robotStateListener->_gl_robot->is_future_state_changing())
    
      
      // set desired state
      KDL::Frame T_world_body_future = self->robotStateListener->_gl_robot->_T_world_body_future;
      double currentAngle, angleTo,dtheta;       
      KDL::Frame DragRotation=KDL::Frame::Identity();       
      if(self->robotStateListener->_gl_robot->is_jointdof_adjustment_enabled())
      {
        //===========================================================================
        // set joint dof

        string link_name = (*self->selection); 
        string marker_name = (*self->marker_selection); 
        string token  = "markers::";
        size_t found = marker_name.find(token);  
        if (found==std::string::npos)
            return;
        string joint_name =marker_name.substr(found+token.size());
        
        //std::cout <<"markername: "<< marker_name<< " mouse on joint marker: " << joint_name << std::endl;


      // Get joint marker draw frame

        
        visualization_utils::JointFrameStruct jointInfo;
        self->robotStateListener->_gl_robot->get_joint_info(joint_name,jointInfo);
        KDL::Frame T_world_body = self->robotStateListener->_gl_robot->_T_world_body;
        KDL::Frame T_world_body_future = self->robotStateListener->_gl_robot->_T_world_body_future;
        
        Eigen::Vector3f joint_axis;
        if(self->robotStateListener->_gl_robot->is_future_display_active())        
          joint_axis << jointInfo.future_axis[0],jointInfo.future_axis[1],jointInfo.future_axis[2];
        else
          joint_axis << jointInfo.axis[0],jointInfo.axis[1],jointInfo.axis[2]; // in world frame
        joint_axis.normalize();
       
        
        Eigen::Vector3f u_body_to_joint;
        KDL::Frame T_world_joint;
        T_world_joint = jointInfo.future_frame;
        u_body_to_joint[0] = T_world_body_future.p[0]-T_world_joint.p[0];
        u_body_to_joint[1] = T_world_body_future.p[1]-T_world_joint.p[1];
        u_body_to_joint[2] = T_world_body_future.p[2]-T_world_joint.p[2];
        u_body_to_joint.normalize();

        
        double normal = acos(u_body_to_joint.dot(joint_axis));
        double flipped = acos(u_body_to_joint.dot(-joint_axis));
        
        KDL::Frame T_world_jointaxis;// Axis
        T_world_jointaxis.p = jointInfo.future_frame.p;

        double theta;
        Eigen::Vector3f axis;      
        Eigen::Vector3f uz; 
        uz << 0 , 0 , 1; 
        axis = uz.cross(joint_axis);
        axis.normalize();
        theta = acos(uz.dot(joint_axis));
        KDL::Vector axis_temp;
        axis_temp[0]=axis[0];axis_temp[1]=axis[1];axis_temp[2]=axis[2];
        T_world_jointaxis.M = KDL::Rotation::Rot(axis_temp,theta); //T_axis_world
        
        KDL::Frame T_world_marker = KDL::Frame::Identity();
        KDL::Frame T_jointaxis_marker = KDL::Frame::Identity();
        double arrow_length =0.2;
        if(flipped>normal+1e-1) {
          T_jointaxis_marker.p[2] =-2*arrow_length/3;
         }
        else{
          T_jointaxis_marker.p[2] = 2*arrow_length/3;
        }
        T_world_marker = T_world_jointaxis*T_jointaxis_marker; // T_axismarker_world = T_axismarker_axis*T_axis_world
        
       // proper hit_drag point via marker plane ray intersection.
        Eigen::Vector3f plane_normal,plane_point;
          
        plane_normal = joint_axis;
        plane_point[0]=T_world_marker.p[0];
        plane_point[1]=T_world_marker.p[1];
        plane_point[2]=T_world_marker.p[2];       
        double lambda1 = dir.dot(plane_normal);
        double lambda2 = (plane_point - start).dot(plane_normal);
        double t;
    
       // check for degenerate case where ray is (more or less) parallel to plane
       if (fabs (lambda1) >= 1e-9) {
         t = lambda2 / lambda1;
          self->ray_hit_drag << start[0]+t*dir[0], start[1]+t*dir[1], start[2]+t*dir[2];  
         }
        // else  no solution        
        
        Eigen::Vector3f diff = self->prev_ray_hit_drag - self->ray_hit_drag; 
        if(diff.norm() > 0.05){ // to avoid run off into horizon ? (Why did I add this? - Sisir)
          self->prev_ray_hit_drag = self->ray_hit_drag; 
        }
        Eigen::Vector3f hit_markerframe,hitdrag_markerframe;
        //convert to joint dof marker frame .
        rotate_eigen_vector_given_kdl_frame(self->prev_ray_hit_drag,T_world_marker.Inverse(),hit_markerframe); 
        rotate_eigen_vector_given_kdl_frame(self->ray_hit_drag,T_world_marker.Inverse(),hitdrag_markerframe); 
 
        int type = jointInfo.type;   
        if((type==otdf::Joint::REVOLUTE)||(type==otdf::Joint::CONTINUOUS))
        {          
          double currentAngle, angleTo, dtheta;         
          currentAngle = atan2(hit_markerframe[1],hit_markerframe[0]);
          angleTo = atan2(hitdrag_markerframe[1],hitdrag_markerframe[0]);
         /* cout << "currentAngle :"<< currentAngle*(180/M_PI)
               << " angleTo :"<< angleTo*(180/M_PI) <<endl;
          cout << "radius" << sqrt(pow(hitdrag_markerframe[0],2)+pow(hitdrag_markerframe[1],2)) << endl; */
          dtheta = gain*shortest_angular_distance(currentAngle,angleTo);
          std::map<std::string, double> jointpos_in;
          jointpos_in = self->robotStateListener->_gl_robot->_future_jointpos;
          jointpos_in.find(joint_name)->second = (jointpos_in.find(joint_name)->second +dtheta); 
          self->robotStateListener->_gl_robot->set_future_state(T_world_body_future,jointpos_in);   
          bot_viewer_request_redraw(self->viewer);
         // bot_gtk_param_widget_set_double(self->pw, PARAM_CURRENT_JOINTPOS, jointpos_in.find(joint_name)->second); // DO NOT SET THIS ON MARKER MOTION
         }// end revolute joints
         else if(type==otdf::Joint::PRISMATIC)
         {
          double s=1;
          if(diff.dot(joint_axis)<0)
            s = -1;
          double distance = s*diff.norm();
           std::map<std::string, double> jointpos_in;
           jointpos_in = self->robotStateListener->_gl_robot->_future_jointpos;
           jointpos_in.find(joint_name)->second -= distance;
           self->robotStateListener->_gl_robot->set_future_state(T_world_body_future,jointpos_in); 
          // bot_gtk_param_widget_set_double(self->pw, PARAM_CURRENT_JOINTPOS, jointpos_in.find(joint_name)->second); // DO NOT SET THIS ON MARKER MOTION
           bot_viewer_request_redraw(self->viewer);  
         }
      }
      
      self->prev_ray_hit_drag = self->ray_hit_drag; 
     bot_viewer_request_redraw(self->viewer);     
  }   // end set_desired_robot_posture_on_marker_motion()
  
  inline static void publish_posture_goal(void *user, const string& channel)
  {
        RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
        drc::joint_angles_t msg;
        msg.utime = self->robotStateListener->_last_state_msg_sim_timestamp;
        msg.robot_name = self->robotStateListener->_robot_name;
        int num_joints = 0;
        typedef map<string,double> jointpos_type;
        for(jointpos_type::iterator it = self->robotStateListener->_gl_robot->_future_jointpos.begin(); it!=self->robotStateListener->_gl_robot->_future_jointpos.end(); it++) 
        { 
          jointpos_type::iterator it2 = self->robotStateListener->_gl_robot->_current_jointpos.find(it->first);
          if(it->second!=it2->second)
          {
             msg.joint_name.push_back(it->first);
             msg.joint_position.push_back(it->second);
             num_joints++; 
          }
        } // end for 
        msg.num_joints =  num_joints; 
        self->lcm->publish(channel, &msg);
   }

  static void set_desired_robot_posture_from_pw(void *user, double value)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    if(!self->robotStateListener->_gl_robot->is_future_state_changing()) {
      self->robotStateListener->_gl_robot->set_future_state_changing(true);  
     }
    KDL::Frame T_world_body_future = self->robotStateListener->_gl_robot->_T_world_body_future;
    if(self->robotStateListener->_gl_robot->is_jointdof_adjustment_enabled())
    {
      string link_name = (*self->selection); 
      string marker_name = (*self->marker_selection); 
      string token  = "markers::";
      size_t found = marker_name.find(token);  
      if (found==std::string::npos)
          return;
      string joint_name =marker_name.substr(found+token.size());
      std::map<std::string, double> jointpos_in;
      jointpos_in = self->robotStateListener->_gl_robot->_future_jointpos;
      jointpos_in.find(joint_name)->second = value; 
      self->robotStateListener->_gl_robot->set_future_state(T_world_body_future,jointpos_in);   
      bot_viewer_request_redraw(self->viewer);
    }
    publish_posture_goal(self, "POSTURE_GOAL");
  }


  // Backwardly compatiable:
  inline static void publish_posture_goal_back_zeroed(void *user)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    if (!(self->zero_back_options.roll | self->zero_back_options.pitch | self->zero_back_options.yaw)) {
      return;
    }
    // Fixing the hands is currently only supported through the drill planner, rather than the posture planner, 
    // so we'll deciede here which planner to use (and thus which channel to transmit on) based on that flag.
    if (!self->zero_back_options.fix_hands) {
      drc::joint_angles_t msg;
      // Hard coded zeroing of joints:
      if (self->zero_back_options.roll) {
        msg.joint_name.push_back("back_bkx");
        msg.joint_position.push_back(0);
      }
      if (self->zero_back_options.pitch) {
        msg.joint_name.push_back("back_bky");
        msg.joint_position.push_back(0);
      }
      if (self->zero_back_options.yaw) {
        msg.joint_name.push_back("back_bkz");
        msg.joint_position.push_back(0);
      }
      msg.num_joints =  msg.joint_name.size(); 
      msg.utime = self->robotStateListener->_last_state_msg_sim_timestamp;
      msg.robot_name = self->robotStateListener->_robot_name;
      self->lcm->publish("POSTURE_GOAL", &msg);
    } else {
      drc::drill_control_t msg;
      msg.control_type = drc::drill_control_t::STRAIGHTEN_BACK_FIXED_HANDS;
      msg.data.push_back((double) self->zero_back_options.roll);
      msg.data.push_back((double) self->zero_back_options.pitch);
      msg.data.push_back((double) self->zero_back_options.yaw);
      msg.data_length = 3;
      msg.utime = self->robotStateListener->_last_state_msg_sim_timestamp;
      self->lcm->publish("DRILL_CONTROL", &msg);
    }
  }
   
  inline static void publish_walking_goal(void *user, const string& channel)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    drc::walking_goal_t msg;
    msg.utime = self->robotStateListener->_last_state_msg_sim_timestamp; //bot_timestamp_now();
    msg.max_num_steps = (int32_t) 15;
    msg.min_num_steps = (int32_t) 2;
    //msg.robot_name = self->robotStateListener->_robot_name; 
    msg.timeout=0;

    msg.goal_pos.translation.x = self->robotStateListener->_gl_robot->_T_world_body_future.p[0];
    msg.goal_pos.translation.y = self->robotStateListener->_gl_robot->_T_world_body_future.p[1];
    msg.goal_pos.translation.z = 0;
    double r,p,y;
    self->robotStateListener->_gl_robot->_T_world_body_future.M.GetRPY(r,p,y);
    double rpy[] = {0,0,y};
    double quat_out[4];
    bot_roll_pitch_yaw_to_quat(rpy, quat_out); // its in w,x,y,z format
    msg.goal_pos.rotation.w = quat_out[0];
    msg.goal_pos.rotation.x = quat_out[1];
    msg.goal_pos.rotation.y = quat_out[2];
    msg.goal_pos.rotation.z = quat_out[3];
    msg.behavior = msg.BEHAVIOR_BDI_STEPPING;;//msg.BEHAVIOR_WALKING;
    
    msg.nom_step_width = 0.26;
    msg.nom_forward_step = 0.25;
    msg.max_forward_step = 0.5;
    msg.is_new_goal = true;
    msg.allow_optimization = false;
    msg.step_speed = 1.0;
    msg.step_height = 0.1;
    msg.mu = 1.00;
    msg.follow_spline = false;
    msg.ignore_terrain = true;
    msg.right_foot_lead = true;

    msg.fixed_step_duration = NAN;
    msg.bdi_step_duration = NAN;
    msg.bdi_sway_duration = NAN;
    msg.bdi_lift_height = NAN;
    msg.bdi_toe_off = 1;
    msg.bdi_knee_nominal = NAN;


    cout << "Sending WALKING_GOAL\n";
    self->lcm->publish(channel, &msg);
 } 
 
  inline static void publish_desired_footstep_sequence(void *user, const string& channel)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    
    drc::traj_opt_constraint_t msg;    
    msg.utime = self->robotStateListener->_last_state_msg_sim_timestamp; //bot_timestamp_now();
    msg.robot_name = self->robotStateListener->_robot_name;
    
    KDL::Frame T_world_ee;
    std::vector<std::string> ee_names;
    ee_names.push_back("l_foot");
    ee_names.push_back("r_foot");
    for (size_t i=0; i<ee_names.size(); i++)
    {
      self->robotStateListener->_gl_robot->get_link_future_frame(ee_names[i],T_world_ee);
      double x,y,z,w;
      T_world_ee.M.GetQuaternion(x,y,z,w);
      drc::position_3d_t pose;
      pose.translation.x = T_world_ee.p[0];
      pose.translation.y = T_world_ee.p[1];
      pose.translation.z = NAN;//T_world_ee.p[2];
      pose.rotation.x = x;
      pose.rotation.y = y;
      pose.rotation.z = z;
      pose.rotation.w = w; 
      msg.link_name.push_back(ee_names[i]);
      msg.link_origin_position.push_back(pose);  
      int64_t time_stamp = (int64_t)i*1000000;
      msg.link_timestamps.push_back(time_stamp);  
    }
    msg.num_links =  ee_names.size();
    msg.num_joints = 0;

    cout << "Sending Desired Foot Step Sequence\n";
    self->lcm->publish(channel, &msg);
 } 
 
  static void publish_candidate_sticky_feet(void *user, const string& channel,string &otdf_uid, KDL::Frame &T_world_aff, bool desired_state)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    
    drc::traj_opt_constraint_t msg;    
    msg.utime = bot_timestamp_now();
    msg.robot_name = otdf_uid;//self->robotStateListener->_robot_name;
    
    KDL::Frame T_world_ee;
    std::vector<std::string> ee_names;
    ee_names.push_back("l_foot");
    ee_names.push_back("r_foot");
    for (size_t i=0; i<ee_names.size(); i++)
    {
      if(desired_state)
        self->robotStateListener->_gl_robot->get_link_future_frame(ee_names[i],T_world_ee);
      else
        self->robotStateListener->_gl_robot->get_link_frame(ee_names[i],T_world_ee);
        
      KDL::Frame T_aff_ee;
      T_aff_ee = T_world_aff.Inverse()*T_world_ee;
      double x,y,z,w;
      T_aff_ee.M.GetQuaternion(x,y,z,w);
      drc::position_3d_t pose;
      pose.translation.x = T_aff_ee.p[0];
      pose.translation.y = T_aff_ee.p[1];
      pose.translation.z = T_aff_ee.p[2];
      pose.rotation.x = x;
      pose.rotation.y = y;
      pose.rotation.z = z;
      pose.rotation.w = w; 
      msg.link_name.push_back(ee_names[i]);
      msg.link_origin_position.push_back(pose);  
      int64_t time_stamp = (int64_t)0*1000000;
      msg.link_timestamps.push_back(time_stamp);  
    }
    msg.num_links =  ee_names.size();
    msg.num_joints = 0;

    cout << "Sending Candidate StickyFeet\n";
    self->lcm->publish(channel, &msg);
 }  
} // end namespace
  
  
void setup_renderer_robot_state(BotViewer *viewer, int render_priority, lcm_t *lcm, int operation_mode, KeyboardSignalRef signalRef,AffTriggerSignalsRef affTriggerSignalsRef,RendererFoviationSignalRef rendererFoviationSignalRef);
#endif //RENDERER_ROBOT_STATE_HPP
