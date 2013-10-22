#ifndef OBJECT_INTERACTION_GUI_UTILS_HPP
#define OBJECT_INTERACTION_GUI_UTILS_HPP

#define PARAM_DIL_FACTOR "Obj Dilation"
#define PARAM_SEED_LH "Seed LHand"
#define PARAM_SEED_RH "Seed RHand"
#define PARAM_SEED_LF "Seed LFoot"
#define PARAM_SEED_RF "Seed RFoot"
#define PARAM_CLEAR_SEEDS "Clear Seeds"
#define PARAM_STORE "Store to otdf file" 
#define PARAM_UNSTORE "Unstore from otdf file" 


#define PARAM_HALT_ALL_OPT "Halt All Opts"

#define PARAM_REACH "Reach" 
#define PARAM_TOUCH "Touch"   // publishes pre-grasp pose as ee_goal for reaching controller

#define PARAM_GRASP_UNGRASP   "Grasp/Ungrasp"  // commits grasp state as setpoint and enables grasp controller
#define PARAM_GRASP   "Grasp" 
#define PARAM_SQUEEZE    "Squeeze"
#define PARAM_UNGRASP   "Ungrasp" 


#define PARAM_POWER_GRASP     "PowerGrasp"
#define PARAM_SEND_POSE_GOAL   "Get Robot Pose @CurAffState"
#define PARAM_SEND_POSE_GOAL2  "Get Robot Pose @DesAffState"
#define PARAM_SEND_POSE_GOAL3  "Get Robot Pose 4DesAffMotion"

#define PARAM_SEND_POSE_GOAL4  "Get Robot Pose @CurState"
#define PARAM_SEND_POSE_GOAL5  "Get Robot Pose @DesState"
#define PARAM_SEND_POSE_GOAL6  "Get Robot Pose 4DesEEMotion"

#define PARAM_MELD_HAND_TO_CURRENT  "Meld::2::CurHndState"
#define PARAM_MELD_FOOT_TO_CURRENT  "Meld::2::CurFootState"
#define PARAM_MELD_PARENT_AFF_TO_ESTROBOTSTATE  "Meld::Aff::2::EstRobotState"
#define PARAM_MATE     "Mate"
#define PARAM_SELECT_EE_TYPE "EE :"
#define PARAM_SELECT_MATE_AXIS_FOR_EE_TELEOP "Mate Axis (FemaleEnd):"
#define PARAM_ENGAGE_EE_TELEOP "Engage EE Teleop"

#define PARAM_PARTIAL_GRASP_UNGRASP   "G"
// publishes grasp pose as ee_goal for reaching controller. Simultaneously grasp controller executes only if ee pose is close to the committed grasp pose (if inFunnel, execute grasp)
#define PARAM_MOVE_EE "Move"

#define PARAM_SET_EE_CONSTRAINT "Set EE Gaze/Orient Constraints"

#define PARAM_RESEED "Re-seed"
#define PARAM_HALT_OPT "Halt Opt"
#define PARAM_DELETE "Delete"
#define PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT "Adjust Cur BodyPose"
#define PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT "Adjust Cur JointDofs"
#define PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT "Set Des BodyPose"
#define PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT "Set Des JointDofs"
#define PARAM_ADJUST_DESIRED_DOFS_VIA_SLIDERS "Adjust (via Sliders)"
#define PARAM_SELECT_FLIP_DIM "Flip Dim"
#define PARAM_FLIP_GEOMETRY "Flip Geometry"
#define PARAM_GET_MANIP_PLAN "Get Manip Plan"
#define PARAM_GET_RETRACTABLE_MANIP_PLAN "Get Retractable Manip Plan"
#define PARAM_GET_MANIP_MAP "Get Manip Map"
#define PARAM_RESET_DESIRED_STATE "Reset"
#define PARAM_HAND_CONTACT_MASK_SELECT "Hand Mask"
#define PARAM_FOOT_CONTACT_MASK_SELECT "Foot Mask"
#define PARAM_EE_SELECT_EE "Select EE"
#define PARAM_EE_HEAD "Head"
#define PARAM_EE_RIGHT_HAND "Right hand"
#define PARAM_EE_LEFT_HAND "Left hand"
#define PARAM_USE_CURRENT_POSE "Use current pose"

#define PARAM_STORE_PLAN "Store Current Plan" 
#define PARAM_PLAN_SEED_LIST "Plan Select" 
#define PARAM_LOAD_PLAN "Load PlanSeed" 
#define PARAM_REACH_STARTING_POSTURE "Reach Starting Posture"
#define PARAM_REACH_STARTING_POSE "Reach Starting Pose"
#define PARAM_UNSTORE_PLAN "Unstore  PlanSeed"
#define PARAM_COMMIT_TO_COLLISION_SERVER "Commit to collision-server"

#define PARAM_EE_SPECIFY_GOAL "Select EE goal"
#define PARAM_CURRENT_ORIENTATION "Maintain EE orientation"
#define PARAM_GAZE_AFFORDANCE "Look at affordance"
//#define PARAM_GAZE_SELECTION" "Look at selected point (NOT SUPPORTED)"
#define PARAM_CURRENT_POSE "Maintain pose"
#define PARAM_CLEAR_CURRENT_GOAL "Clear EE goal"

#define PARAM_SEND_EE_GOAL_SEQUENCE "Get Whole Body Plan"
#define PARAM_COMMIT_TO_FOOTSTEP_PLANNER "Commit to Footstep Planner"

#include "renderer_affordances.hpp"
#include "otdf_instance_management_gui_utils.hpp"
#include "lcm_utils.hpp"

//#include "AffordanceCollectionListener.hpp"
#include "RobotStateListener.hpp"
#include "InitGraspOptPublisher.hpp"
#include "CandidateGraspSeedListener.hpp"
#include "GraspOptStatusListener.hpp"
//#include "ReachabilityVerifier.hpp"

using namespace renderer_affordances;
using namespace renderer_affordances_lcm_utils;


typedef enum _ee_type_t {
    EE_HEAD, EE_RIGHT_HAND, EE_LEFT_HAND
} ee_type_t;

typedef enum _ee_goal_type_t {
    CURRENT_POSE,CURRENT_ORIENTATION,GAZE,CLEAR_CURRENT_GOAL 
} ee_goal_type_t;

namespace renderer_affordances_gui_utils
{
//--------------------------------------------------------------------------------
//  OTDF object dblclk popup and associated cbs.

   static void on_ee_goal_widget_closed(BotGtkParamWidget *pw, const void *user)
    {
        RendererAffordances *self = (RendererAffordances*) user;
      
        ee_type_t ee_type = (ee_type_t) bot_gtk_param_widget_get_enum(pw, PARAM_EE_SELECT_EE);
        ee_goal_type_t ee_goal_type = (ee_goal_type_t) bot_gtk_param_widget_get_enum(pw, PARAM_EE_SPECIFY_GOAL);

        if (ee_goal_type == CURRENT_POSE) {
            fprintf(stderr, "Enum : %d - Goal type : %d\n", ee_type, ee_goal_type);
            BotTrans ee_to_local;
            if (ee_type == EE_HEAD)
            {
              if(self->frames) {
                 bot_frames_get_trans(self->frames, "head", "local", &ee_to_local);
                 fprintf(stderr, "EE Pose : %f,%f,%f\n", ee_to_local.trans_vec[0], ee_to_local.trans_vec[1], ee_to_local.trans_vec[2]);
                 publish_ee_goal_to_gaze(self->lcm, "head", "HEAD_GOAL", ee_to_local);
              }
              else {
                 fprintf(stderr, "BotFrames is null - Unable to find ee pose\n");
              }
            }
            else if(ee_type == EE_RIGHT_HAND){
             BotTrans temp;
             KDL::Frame T_world_palm = KDL::Frame::Identity();
              if(self->robotStateListener->_urdf_parsed){
                self->robotStateListener->_gl_robot->get_link_frame("right_palm",T_world_palm);
                double x,y,z,w;
                T_world_palm.M.GetQuaternion(x,y,z,w);
                temp.rot_quat[0] = w; temp.rot_quat[1] = x; temp.rot_quat[2] = y; temp.rot_quat[3] = z; 
                temp.trans_vec[0] = T_world_palm.p[0]; temp.trans_vec[1] = T_world_palm.p[1]; temp.trans_vec[2] = T_world_palm.p[2];
                publish_ee_goal_to_gaze(self->lcm, "right_palm", "RIGHT_PALM_GOAL", temp);  
              }
            }
            else if(ee_type == EE_LEFT_HAND){
              BotTrans temp;
              KDL::Frame T_world_palm = KDL::Frame::Identity();
              if(self->robotStateListener->_urdf_parsed){
                self->robotStateListener->_gl_robot->get_link_frame("left_palm",T_world_palm);
                double x,y,z,w;
                T_world_palm.M.GetQuaternion(x,y,z,w);
                temp.rot_quat[0] = w; temp.rot_quat[1] = x; temp.rot_quat[2] = y; temp.rot_quat[3] = z; 
                temp.trans_vec[0] = T_world_palm.p[0]; temp.trans_vec[1] = T_world_palm.p[1]; temp.trans_vec[2] = T_world_palm.p[2];
                publish_ee_goal_to_gaze(self->lcm, "left_palm", "LEFT_PALM_GOAL", temp);  
              }
            }  
       }
       else if (ee_goal_type == CURRENT_ORIENTATION) {
            BotTrans temp;
            temp.rot_quat[0] = 0; temp.rot_quat[1] = 0; temp.rot_quat[2] = 0; temp.rot_quat[3] = 0; 
            temp.trans_vec[0] = 0; temp.trans_vec[1] = 0; temp.trans_vec[2] = 0;
            
            if (ee_type == EE_HEAD) {
              BotTrans ee_to_local;
              bot_frames_get_trans(self->frames, "head", "local", &ee_to_local);
              fprintf(stderr, "Head Orientation: %f,%f,%f,%f\n", ee_to_local.rot_quat[0], ee_to_local.rot_quat[1], ee_to_local.rot_quat[2], ee_to_local.rot_quat[3]);
              ee_to_local.trans_vec[0] = 0; ee_to_local.trans_vec[1] = 0; ee_to_local.trans_vec[2] = 0;
              publish_ee_goal_to_gaze(self->lcm, "head", "HEAD_ORIENTATION_GOAL", ee_to_local);
            }
            else if(ee_type == EE_RIGHT_HAND) {
              KDL::Frame T_world_palm = KDL::Frame::Identity();
              if(self->robotStateListener->_urdf_parsed)
                self->robotStateListener->_gl_robot->get_link_frame("right_palm",T_world_palm);
              double x,y,z,w;
              T_world_palm.M.GetQuaternion(x,y,z,w);
              temp.rot_quat[0] = w; temp.rot_quat[1] = x; temp.rot_quat[2] = y; temp.rot_quat[3] = z; 
              temp.trans_vec[0] = T_world_palm.p[0]; temp.trans_vec[1] = T_world_palm.p[1]; temp.trans_vec[2] = T_world_palm.p[2];
              publish_ee_goal_to_gaze(self->lcm, "right_palm", "RIGHT_PALM_ORIENTATION_GOAL", temp);  
            }
            else if(ee_type == EE_LEFT_HAND){
              KDL::Frame T_world_palm = KDL::Frame::Identity();
              if(self->robotStateListener->_urdf_parsed)
                self->robotStateListener->_gl_robot->get_link_frame("left_palm",T_world_palm);
              double x,y,z,w;
              T_world_palm.M.GetQuaternion(x,y,z,w);
              temp.rot_quat[0] = w; temp.rot_quat[1] = x; temp.rot_quat[2] = y; temp.rot_quat[3] = z; 
              temp.trans_vec[0] = T_world_palm.p[0]; temp.trans_vec[1] = T_world_palm.p[1]; temp.trans_vec[2] = T_world_palm.p[2];
              publish_ee_goal_to_gaze(self->lcm, "left_palm", "LEFT_PALM_ORIENTATION_GOAL", temp); 
            }
        }
        else if (ee_goal_type == GAZE) {
            BotTrans temp;
            temp.rot_quat[0] = 1; temp.rot_quat[1] = 0; temp.rot_quat[2] = 0; temp.rot_quat[3] = 0; 
            temp.trans_vec[0] = 0; temp.trans_vec[1] = 0; temp.trans_vec[2] = 0;
            
            //Get affordance origin
            typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
            object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(self->object_selection));
            KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;
            temp.trans_vec[0] = T_world_object.p[0]; temp.trans_vec[1] = T_world_object.p[1]; temp.trans_vec[2] = T_world_object.p[2];
            double x,y,z,w;
            T_world_object.M.GetQuaternion(x,y,z,w);
            temp.rot_quat[0] = w; temp.rot_quat[1] = x; temp.rot_quat[2] = y; temp.rot_quat[3] = z;  
            
            if (ee_type == EE_HEAD) {
              publish_ee_goal_to_gaze(self->lcm, "head", "HEAD_GAZE_GOAL", temp);
            }
            else if(ee_type == EE_RIGHT_HAND)  {
              publish_ee_goal_to_gaze(self->lcm, "right_palm", "RIGHT_PALM_GAZE_GOAL", temp);  
            }
            else if(ee_type == EE_LEFT_HAND) {
              publish_ee_goal_to_gaze(self->lcm, "left_palm", "LEFT_PALM_GAZE_GOAL", temp);  
            }
        }
        else if (ee_goal_type == CLEAR_CURRENT_GOAL) {
            BotTrans temp;
            temp.rot_quat[0] = 0; temp.rot_quat[1] = 0; temp.rot_quat[2] = 0; temp.rot_quat[3] = 0; 
            temp.trans_vec[0] = 0; temp.trans_vec[1] = 0; temp.trans_vec[2] = 0;
            
            if (ee_type == EE_HEAD) {
              publish_ee_goal_to_gaze(self->lcm, "head", "HEAD_GOAL_CLEAR", temp);
            }
            else if(ee_type == EE_RIGHT_HAND) {
              publish_ee_goal_to_gaze(self->lcm, "right_palm", "RIGHT_PALM_GOAL_CLEAR", temp); 
            }
            else if(ee_type == EE_LEFT_HAND) {
              publish_ee_goal_to_gaze(self->lcm, "left_palm", "LEFT_PALM_GOAL_CLEAR", temp);  
            }
        }
    }    
    
//------------------------------------------------------------------     
   
     static void spawn_get_ee_constraint_popup (RendererAffordances *self)
    {

        GtkWidget *window, *close_button, *vbox;
        BotGtkParamWidget *pw;

        window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
        gtk_window_set_modal(GTK_WINDOW(window), FALSE);
        gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
        gtk_window_stick(GTK_WINDOW(window));
        gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
        gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
        gint pos_x, pos_y;
        gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
        pos_x+=125;    pos_y-=75;
        gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
        //gtk_widget_set_size_request (window, 300, 250);
        //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
        gtk_window_set_title(GTK_WINDOW(window), "Set EE");
        gtk_container_set_border_width(GTK_CONTAINER(window), 5);
        pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

        bot_gtk_param_widget_add_enum(pw, PARAM_EE_SELECT_EE, BOT_GTK_PARAM_WIDGET_MENU, EE_HEAD, 
                                      PARAM_EE_HEAD, EE_HEAD, 
                                      PARAM_EE_RIGHT_HAND, EE_RIGHT_HAND, 
                                      PARAM_EE_LEFT_HAND, EE_LEFT_HAND, 
                                      NULL);
    
        bot_gtk_param_widget_add_enum(pw, PARAM_EE_SPECIFY_GOAL, BOT_GTK_PARAM_WIDGET_MENU, GAZE, 
                                      PARAM_GAZE_AFFORDANCE, GAZE,
                                      PARAM_CURRENT_ORIENTATION, CURRENT_ORIENTATION, 
                                      PARAM_CURRENT_POSE, CURRENT_POSE, 
                                      PARAM_CLEAR_CURRENT_GOAL, CLEAR_CURRENT_GOAL,
                                      NULL);

        //bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_USE_CURRENT_POSE, 1, NULL);

        close_button = gtk_button_new_with_label ("Close");

        g_signal_connect (G_OBJECT (close_button),
                          "clicked",
                          G_CALLBACK (on_popup_close),
                          (gpointer) window);
        g_signal_connect(G_OBJECT(pw), "destroy",
                         G_CALLBACK(on_ee_goal_widget_closed), self); 

        vbox = gtk_vbox_new (FALSE, 3);
        gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
        gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
        gtk_container_add (GTK_CONTAINER (window), vbox);
        gtk_widget_show_all(window); 
    } 
  

//---------------------------------------------------------------
// SECOND STAGE ADJUST DOFS POPUP
  static void on_adjust_dofs_popup_close2 (BotGtkParamWidget *pw, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    std::string instance_name=  self->object_selection;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->affCollection->_objects.find(instance_name);
    self->motion_trail_log_enabled =false;
    it->second._gl_object->set_future_state_changing(false);
    self->second_stage_popup  = NULL;
  //   TODO: Send publish desired affordance state command msg    
  
  }

  static void on_otdf_adjust_dofs_widget_changed2(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    std::string instance_name=  self->object_selection;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->affCollection->_objects.find(instance_name);
    KDL::Frame T_world_object = it->second._gl_object->_T_world_body;

      
    if(!it->second._gl_object->is_future_state_changing()) {
       std::map<std::string, double> jointpos_in;
      jointpos_in = it->second._gl_object->_future_jointpos;
    
      it->second._gl_object->set_future_state(T_world_object,jointpos_in);
      it->second._gl_object->set_future_state_changing(true);

     // clear previously accumulated motion states for all dependent bodies
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.begin();
      while (hand_it!=self->stickyHandCollection->_hands.end()) 
      {
         if (hand_it->second.object_name == (instance_name))
         {
            hand_it->second._gl_hand->clear_desired_body_motion_history();
         }
         hand_it++;
      }
     }//end if(!it->second._gl_object->is_future_state_changing())

    // get desired state from popup sliders
    if(it->second._otdf_instance->root_link_->name!= "world")// object is not bolted to the world.
    {
        T_world_object.p[0]   =  bot_gtk_param_widget_get_double (pw, "x");
        T_world_object.p[1]   =  bot_gtk_param_widget_get_double (pw, "y");
        T_world_object.p[2]   =  bot_gtk_param_widget_get_double (pw, "z");
        double desired_roll,desired_pitch, desired_yaw;
        desired_roll   =  bot_gtk_param_widget_get_double (pw, "roll")*(M_PI/180);
        desired_pitch   =  bot_gtk_param_widget_get_double (pw, "pitch")*(M_PI/180);
        desired_yaw   =  bot_gtk_param_widget_get_double (pw, "yaw")*(M_PI/180);  
        T_world_object.M = KDL::Rotation::RPY(desired_roll,desired_pitch,desired_yaw);
    }
    
    
    std::map<std::string, double> jointpos_in;
      
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double desired_dof_pos = 0; // get current desired dof pos.
      if(joint->second->type!=(int) otdf::Joint::FIXED) {
          desired_dof_pos =  bot_gtk_param_widget_get_double (pw, joint->first.c_str())*(M_PI/180);
          jointpos_in.insert(make_pair(joint->first, desired_dof_pos)); 
       cout <<  joint->first << " dof changed to " << desired_dof_pos*(180/M_PI) << endl;
      }
     }
    it->second._gl_object->set_future_state(T_world_object,jointpos_in); 

    bot_viewer_request_redraw(self->viewer);
  }
  
  static void spawn_adjust_dofs_popup_2 (RendererAffordances *self)
  {

    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw;

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    gint pos_x, pos_y;
    gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
    pos_x+=175;    pos_y-=75;
   // gint gdk_screen_height  (void);//Returns the height of the default screen in pixels.
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "Adjust Desired DOFs");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    std::string instance_name=  self->object_selection;

    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->affCollection->_objects.find(instance_name);
    self->motion_trail_log_enabled =true;
    if(!it->second._gl_object->is_future_state_changing()) 
       it->second._gl_object->set_future_state_changing(true); 
    
    KDL::Frame T_world_object = it->second._gl_object->_T_world_body_future;
    double current_roll,current_pitch, current_yaw;
    T_world_object.M.GetRPY(current_roll,current_pitch,current_yaw);
    // check root link
    // if root link is not world
    // add x,y,z,roll,pitch,yaw sliders.
    if(it->second._otdf_instance->root_link_->name!= "world")// object is not bolted to the world.
    {
        bot_gtk_param_widget_add_double(pw, "x", BOT_GTK_PARAM_WIDGET_SLIDER, T_world_object.p[0]-1, T_world_object.p[0]+1, .01, T_world_object.p[0]); 
        bot_gtk_param_widget_add_double(pw, "y", BOT_GTK_PARAM_WIDGET_SLIDER, T_world_object.p[1]-1, T_world_object.p[1]+1, .01, T_world_object.p[1]);
        bot_gtk_param_widget_add_double(pw, "z", BOT_GTK_PARAM_WIDGET_SLIDER, T_world_object.p[2]-1, T_world_object.p[2]+1, .01, T_world_object.p[2]);
        bot_gtk_param_widget_add_double(pw, "roll", BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_roll*(180/M_PI)); 
        bot_gtk_param_widget_add_double(pw, "pitch", BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_pitch*(180/M_PI)); 
        bot_gtk_param_widget_add_double(pw, "yaw", BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_yaw*(180/M_PI)); 
    }

    // Need tracked joint positions of all objects.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double current_dof_position = it->second._gl_object->_future_jointpos.find(joint->first)->second;
      if(joint->second->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
        if(joint->second->type==(int) otdf::Joint::CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
        }
        else{
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          joint->second->limits->lower*(180/M_PI), joint->second->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
        }   
      }
    }
    //Have to handle joint_patterns separately   
    // DoF of all joints in joint patterns.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
    for (jp_mapType::iterator jp_it = it->second._otdf_instance->joint_patterns_.begin();jp_it != it->second._otdf_instance->joint_patterns_.end(); jp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
      {
        double current_dof_position = it->second._gl_object->_future_jointpos.find(jp_it->first)->second;
        if(jp_it->second->joint_set[i]->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
          if(jp_it->second->joint_set[i]->type==(int) otdf::Joint::CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
          }
          else{
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          jp_it->second->joint_set[i]->limits->lower*(180/M_PI), jp_it->second->joint_set[i]->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
          }   
        } // end if         
      } // end for all joints in jp
    }// for all joint patterns

    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_adjust_dofs_widget_changed2), self);


    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),
                    "clicked",
                    G_CALLBACK (on_popup_close),
                    (gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy",
      G_CALLBACK(on_adjust_dofs_popup_close2), self); 

    self->second_stage_popup  = window;
    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 
  }




//---------------------------------------------------------------
// FIRST STAGE POPUP
//---------------------------------------------------------------

  static void on_object_geometry_dblclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it= self->affCollection->_objects.find(self->object_selection);

    if(!strcmp(name,PARAM_OTDF_DELETE)) {
      fprintf(stderr,"\nClearing Selected Affordance\n");
      if(it!=self->affCollection->_objects.end())
      {
        string instance_name = self->object_selection;
        self->affCollection->delete_otdf_from_affstore("AFFORDANCE_FIT", it->second.otdf_type, it->second.uid);

        self->affCollection->_objects.erase(it);
        if(self->object_selection==string(instance_name))
        {
            self->link_selection = " ";
            self->object_selection = " ";
        }  

        typedef map<string, StickyHandStruc > sticky_hands_map_type_;
        sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.begin();
        while (hand_it!=self->stickyHandCollection->_hands.end()) {
          string hand_name = string(hand_it->second.object_name);
          if (hand_name == string(instance_name))
          {
            if(self->stickyhand_selection==hand_it->first){
              self->seedSelectionManager->remove(self->stickyhand_selection);
              self->stickyhand_selection = " ";
            }
            self->stickyHandCollection->_hands.erase(hand_it++);
          }
          else
            hand_it++;
        } // end while

      typedef map<string, StickyFootStruc > sticky_feet_map_type_;
      sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.begin();
      while (foot_it!=self->stickyFootCollection->_feet.end()) {
        string foot_name = string(foot_it->second.object_name);
        if (foot_name == string(instance_name))
        {
          if(self->stickyfoot_selection==foot_it->first)
          {
            self->seedSelectionManager->remove(self->stickyfoot_selection);                            
            self->stickyfoot_selection = " ";
          }
          self->stickyFootCollection->_feet.erase(foot_it++);
        }
        else
          foot_it++;
      } // end while

      bot_viewer_request_redraw(self->viewer);
      }// end if
    }
    else if (! strcmp(name, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT)) {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT);
      if(val){
        bot_gtk_param_widget_set_bool(pw, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT,false); 
        std::cout << "enabling bodypose adjustment for object " <<self->object_selection << std::endl;
        self->motion_trail_log_enabled =true;
      }
     else{
         std::cout << "disabling bodypose adjustment for object " <<self->object_selection << std::endl;
         bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT);
         if((self->marker_selection  != " ")&&(!val)) {
          if(it->second._gl_object->is_future_state_changing())
            it->second._gl_object->set_future_state_changing(false);   // when both jointdof and bodypose are disabled.
         }
         self->motion_trail_log_enabled =false;
      }  
        
      if(it!=self->affCollection->_objects.end()){
        it->second._gl_object->enable_bodypose_adjustment(val);
        it->second._gl_object->enable_bodyorparent_frame_rendering_of_floatingbase_markers(val); 
        it->second._gl_object->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::THREE_D);
        it->second._gl_object->enable_jointdof_adjustment(false);    
      }
    }
    else if (! strcmp(name, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT)) {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT);
      if(val){
        bot_gtk_param_widget_set_bool(pw, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT,false);
        std::cout << "enabling jointdof adjustment for object " <<self->object_selection << " to "<< val << std::endl;
        self->motion_trail_log_enabled =true;
      }
      else{
          std::cout << "disabling jointdof adjustment for object " <<self->object_selection << std::endl;
         bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT);
         if((self->marker_selection  != " ")&&(!val)) {
          if(it->second._gl_object->is_future_state_changing())
            it->second._gl_object->set_future_state_changing(false);   // clear future state when both jointdof and bodypose are disabled.
         }
         self->motion_trail_log_enabled =false;
      }
      if(it!=self->affCollection->_objects.end()){
          it->second._gl_object->enable_bodypose_adjustment(false); 
          it->second._gl_object->enable_jointdof_adjustment(val);  
       }      
    }
    else if (! strcmp(name, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT)) {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT);
      if(val){
        bot_gtk_param_widget_set_bool(pw, PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT,false); 
        std::cout << "enabling bodypose adjustment for object " <<self->object_selection << std::endl;
        
        if(!self->selection_hold_on) { // Assuming only one object instance is changed at any given time
          self->otdf_instance_hold.uid=it->second.uid;
          self->otdf_instance_hold.otdf_type = it->second.otdf_type;
          self->otdf_instance_hold._otdf_instance = otdf::duplicateOTDFInstance(it->second._otdf_instance);
          self->otdf_instance_hold._gl_object.reset();
          self->otdf_instance_hold._collision_detector.reset();
          self->otdf_instance_hold._collision_detector = boost::shared_ptr<Collision_Detector>(new Collision_Detector());     
          self->otdf_instance_hold._gl_object = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(self->otdf_instance_hold._otdf_instance,self->otdf_instance_hold._collision_detector,true,"otdf_instance_hold"));
          self->otdf_instance_hold._otdf_instance->update();
          self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
          self->otdf_instance_hold._gl_object->triangles = it->second._gl_object->triangles;
          self->otdf_instance_hold._gl_object->points = it->second._gl_object->points;
          self->otdf_instance_hold._gl_object->isShowMeshSelected = it->second._gl_object->isShowMeshSelected;
          self->selection_hold_on=true;
        }
        
      }
     else{
        std::cout << "disabling bodypose adjustment for object " <<self->object_selection << std::endl;
        self->selection_hold_on=false;
      }  
      if(it!=self->affCollection->_objects.end()){
        self->otdf_instance_hold._gl_object->enable_bodypose_adjustment(val);
        self->otdf_instance_hold._gl_object->enable_bodyorparent_frame_rendering_of_floatingbase_markers(val);
        self->otdf_instance_hold._gl_object->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::THREE_D);
        self->otdf_instance_hold._gl_object->enable_jointdof_adjustment(false);    
      }

    }
    else if (! strcmp(name, PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT)) {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT);
      if(val){
        bot_gtk_param_widget_set_bool(pw, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT,false);
        std::cout << "enabling jointdof adjustment for object " <<self->object_selection << " to "<< val << std::endl;
        
        if(!self->selection_hold_on) { // Assuming only one object instance is changed at any given time
          self->otdf_instance_hold.uid=it->second.uid;
          self->otdf_instance_hold.otdf_type = it->second.otdf_type;  
          //self->otdf_instance_hold.otdf_type = new string((*it->second.otdf_type));    // SEGFAULTS With Strings
          self->otdf_instance_hold._otdf_instance = otdf::duplicateOTDFInstance(it->second._otdf_instance);
          self->otdf_instance_hold._gl_object.reset();
          self->otdf_instance_hold._collision_detector.reset();
          self->otdf_instance_hold._collision_detector = boost::shared_ptr<Collision_Detector>(new Collision_Detector());     
          self->otdf_instance_hold._gl_object = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(self->otdf_instance_hold._otdf_instance,self->otdf_instance_hold._collision_detector,true,"otdf_instance_hold"));
          self->otdf_instance_hold._otdf_instance->update();
          self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
          self->otdf_instance_hold._gl_object->triangles = it->second._gl_object->triangles;
          self->otdf_instance_hold._gl_object->points = it->second._gl_object->points;
          self->otdf_instance_hold._gl_object->isShowMeshSelected = it->second._gl_object->isShowMeshSelected;
          self->selection_hold_on=true;
        }
        
      }
      else{
          std::cout << "disabling jointdof adjustment for object " <<self->object_selection << std::endl;
          self->selection_hold_on=false;
      }
      
      if(it!=self->affCollection->_objects.end()){
         self->otdf_instance_hold._gl_object->enable_bodypose_adjustment(false); 
         self->otdf_instance_hold._gl_object->enable_jointdof_adjustment(val);  
       } 

    } 
    else if(! strcmp(name, PARAM_RESET_DESIRED_STATE)) {
       it->second._gl_object->set_future_state(it->second._gl_object->_T_world_body,it->second._gl_object->_current_jointpos);   
       it->second._gl_object->disable_future_display(); 
      // clear previously accumulated motion states for all dependent bodies
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.begin();
      while (hand_it!=self->stickyHandCollection->_hands.end()) 
      {
         std::string hand_name = std::string(hand_it->second.object_name);
         if (hand_name == (it->first))
         {
            hand_it->second._gl_hand->clear_desired_body_motion_history();
         }
         hand_it++;
      }

      typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
      sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.begin();
      while (foot_it!=self->stickyFootCollection->_feet.end()) 
      {
         std::string foot_name = std::string(foot_it->second.object_name);
         if (foot_name == (it->first))
         {
            foot_it->second._gl_foot->clear_desired_body_motion_history();
         }
         foot_it++;
      }
       
       bot_viewer_request_redraw(self->viewer);
    } 
    else if ((!strcmp(name, PARAM_SEED_LH))||(!strcmp(name, PARAM_SEED_RH))) {
      bool is_sandia_left = (self->urdf_filenames[self->lhand_urdf_id]=="sandia_hand_left");
      bool is_sandia_right = (self->urdf_filenames[self->rhand_urdf_id]=="sandia_hand_right");

      drc::grasp_opt_control_t msg;
      int grasp_type;
      KDL::Frame T_geom_lhandpose = KDL::Frame::Identity();
      KDL::Frame T_geom_rhandpose = KDL::Frame::Identity(); 
      
      if(!strcmp(name, PARAM_SEED_LH))
      {
        if(is_sandia_left){
          grasp_type = msg.SANDIA_LEFT;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;
          T_geom_lhandpose = self->T_graspgeometry_lhandinitpos_sandia;
        }
        else {
          grasp_type = msg.IROBOT_LEFT;
          T_geom_lhandpose = self->T_graspgeometry_lhandinitpos_irobot;
        }
        
      }
      else if(!strcmp(name, PARAM_SEED_RH))
      {
        if(is_sandia_right){
         grasp_type = msg.SANDIA_RIGHT;//or SANDIA_LEFT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;
           T_geom_rhandpose = self->T_graspgeometry_rhandinitpos_sandia;
        }
        else{
          grasp_type = msg.IROBOT_RIGHT; 
          T_geom_rhandpose = self->T_graspgeometry_rhandinitpos_irobot;
        }
      }
      
      /*
      // NBNBNB mfallon modification to publish xyz location of visual intersection instead of grasp hand pose
      T_geom_lhandpose.p[0] = self->ray_hit(0);
      T_geom_lhandpose.p[1] = self->ray_hit(1);
      T_geom_lhandpose.p[2] = self->ray_hit(2);      
      */
      
      int contact_mask = bot_gtk_param_widget_get_enum (pw, PARAM_HAND_CONTACT_MASK_SELECT);  
      int drake_control =msg.NEW;//or NEW=0, RESET=1, HALT=2;
      self->stickyHandCollection->free_running_sticky_hand_cnt++;
      int uid = self->stickyHandCollection->free_running_sticky_hand_cnt;
       std::string channel;
       if(self->graspOptStatusListener->isOptPoolReady())
       {
          int id =  self->graspOptStatusListener->getNextAvailableOptChannelId();
          if((id!=-1)&&(self->graspOptStatusListener->reserveOptChannel(id,uid)))
          {
           //int id =  1;
            std::stringstream oss;
            oss << "INIT_GRASP_OPT_" << id; 
            channel = oss.str();
            std::cout << channel << "  id :" << id << std::endl;
            double dilation_factor = bot_gtk_param_widget_get_double(pw,PARAM_DIL_FACTOR);
            self->initGraspOptPublisher->publishGraspOptControlMsg(channel,T_geom_lhandpose,T_geom_rhandpose,grasp_type,contact_mask,drake_control,uid,dilation_factor);
           }             
       }
    }
    else if ((!strcmp(name, PARAM_SEED_LF))||(!strcmp(name, PARAM_SEED_RF))) {
      //self->stickyFootCollection->free_running_sticky_foot_cnt++;
      //int uid = self->stickyFootCollection->free_running_sticky_foot_cnt;
      foot_contact_mask_type_t contact_mask = (foot_contact_mask_type_t) bot_gtk_param_widget_get_enum (pw, PARAM_FOOT_CONTACT_MASK_SELECT);
      int foot_type = 0;
      if(!strcmp(name, PARAM_SEED_RF))
      {
       foot_type = 1;
      }
      std::string object_name =self->object_selection;
      
      std::string object_geometry_name = self->link_selection;
      std::string object_name_token  = object_name + "_";
      size_t found = object_geometry_name.find(object_name_token);  
      std::string geometry_name =object_geometry_name.substr(found+object_name_token.size());
      
      // Query Normal at point of dbl click. Drag direction gives foot direction.
      // Foot frame Z direction should point towards normal. 
      // also increments free_running_counter
      self->stickyFootCollection->seed_foot(it->second,object_name,geometry_name,foot_type,contact_mask,self->ray_hit_drag,self->ray_hit,self->ray_hit_normal);

    }
    else if (! strcmp(name, PARAM_FLIP_GEOMETRY)) {
    
      std::string object_name =self->object_selection;
      std::string object_geometry_name = self->link_selection;
      std::string object_name_token  = object_name + "_";
      size_t found = object_geometry_name.find(object_name_token);  
      std::string geometry_name =object_geometry_name.substr(found+object_name_token.size());
      
      std::cout << geometry_name << std::endl;
      
      if(!self->selection_hold_on) { // Assuming only one object instance is changed at any given time
          self->otdf_instance_hold.uid=it->second.uid;
          self->otdf_instance_hold.otdf_type = it->second.otdf_type;
          self->otdf_instance_hold._otdf_instance = otdf::duplicateOTDFInstance(it->second._otdf_instance);
          self->otdf_instance_hold._gl_object.reset();
          self->otdf_instance_hold._collision_detector.reset();
          self->otdf_instance_hold._collision_detector = boost::shared_ptr<Collision_Detector>(new Collision_Detector());     
          self->otdf_instance_hold._gl_object = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(self->otdf_instance_hold._otdf_instance,self->otdf_instance_hold._collision_detector,true,"otdf_instance_hold"));
          self->otdf_instance_hold._otdf_instance->update();
          self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
          self->otdf_instance_hold._gl_object->triangles = it->second._gl_object->triangles;
          self->otdf_instance_hold._gl_object->points = it->second._gl_object->points;
          self->otdf_instance_hold._gl_object->isShowMeshSelected = it->second._gl_object->isShowMeshSelected;
          self->selection_hold_on=true;
        }
      
      
      KDL::Frame T_world_body,T_world_geometry,T_world_body_new;
      KDL::Frame T_body_geometry,T_body_geometry_new;
      it->second._gl_object->get_link_frame(object_geometry_name,T_world_geometry);
      T_world_body = it->second._gl_object->_T_world_body;
      
      T_body_geometry = (T_world_body.Inverse())*T_world_geometry;
      double  roll,pitch,yaw;
      //cout << "bodyframe rpy: " << roll << " " << pitch<< " " << yaw << endl;
      T_body_geometry.M.GetRPY(roll,pitch,yaw);
      T_body_geometry_new = T_body_geometry;
      
      if(bot_gtk_param_widget_get_enum(pw, PARAM_SELECT_FLIP_DIM)==2)
        T_body_geometry_new.M = KDL::Rotation::RPY(roll,pitch,yaw+M_PI); 
      else if(bot_gtk_param_widget_get_enum(pw, PARAM_SELECT_FLIP_DIM)==1)
        T_body_geometry_new.M = KDL::Rotation::RPY(roll,pitch+M_PI,yaw);
      else if(bot_gtk_param_widget_get_enum(pw, PARAM_SELECT_FLIP_DIM)==0)
        T_body_geometry_new.M = KDL::Rotation::RPY(roll+M_PI,pitch,yaw); 
             
      T_world_body_new = (T_body_geometry_new*T_world_geometry.Inverse()).Inverse();
      T_world_body_new.M.GetRPY(roll,pitch,yaw);
      T_world_body_new.p = T_world_body.p; // dont change position

      self->otdf_instance_hold._otdf_instance->setParam("roll", roll);
      self->otdf_instance_hold._otdf_instance->setParam("pitch", pitch);
      self->otdf_instance_hold._otdf_instance->setParam("yaw", yaw);
       
      self->otdf_instance_hold._otdf_instance->update();
      self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
      self->affCollection->publish_otdf_instance_to_affstore("AFFORDANCE_TRACK",(self->otdf_instance_hold.otdf_type),self->otdf_instance_hold.uid,self->otdf_instance_hold._otdf_instance); 
      self->selection_hold_on=false;
    
    }
    else if (! strcmp(name, PARAM_CLEAR_SEEDS)) {
    
     self->stickyHandCollection->remove_seeds(self->object_selection,self->affCollection);
     self->stickyFootCollection->remove_seeds(self->object_selection,self->affCollection);
     size_t found;
     found = self->stickyhand_selection.find(self->object_selection);
     if(found!=std::string::npos){
      self->stickyhand_selection = " ";
     }
     found = self->stickyfoot_selection.find(self->object_selection);
     if(found!=std::string::npos){
      self->stickyfoot_selection = " ";
      }
      self->selection_hold_on = false;
    
    }
    else if (! strcmp(name, PARAM_ADJUST_DESIRED_DOFS_VIA_SLIDERS)) {
        spawn_adjust_dofs_popup_2(self);
    }
    else if (! strcmp(name, PARAM_STORE_PLAN)) {
        // Send aff trigger signal to plan renderer to store current active plan to otdf associated xml file
        (*self->affTriggerSignalsRef).plan_store(it->second.otdf_type,it->second._gl_object->_T_world_body);
    }
    else if (! strcmp(name, PARAM_LOAD_PLAN)) {
        // Send aff trigger signal to plan renderer to load a plan from plan storage 
        string selected_plan;
        selected_plan = self->_planseeds[bot_gtk_param_widget_get_enum(pw, PARAM_PLAN_SEED_LIST)];
        (*self->affTriggerSignalsRef).plan_load(it->second.otdf_type,it->second._gl_object->_T_world_body,selected_plan);
    }
    else if ((! strcmp(name,PARAM_REACH_STARTING_POSTURE))||(! strcmp(name,PARAM_REACH_STARTING_POSE)))
    {
      string selected_plan;
      selected_plan = self->_planseeds[bot_gtk_param_widget_get_enum(pw, PARAM_PLAN_SEED_LIST)];
      KDL::Frame T_world_aff;
      T_world_aff = it->second._gl_object->_T_world_body;
      std::string otdf_id = it->second.otdf_type;
      std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
      std::string otdf_filepath,plan_xml_dirpath;
      otdf_filepath =  otdf_models_path + otdf_id +".otdf";
      plan_xml_dirpath =  otdf_models_path + "stored_plans/";
      PlanSeed planSeed;
      planSeed.loadFromOTDF(otdf_filepath,plan_xml_dirpath,selected_plan);
      
      if(! strcmp(name,PARAM_REACH_STARTING_POSTURE)){
      drc::joint_angles_t posture_goal_msg;
      visualization_utils::getFirstFrameInPlanAsPostureGoal(T_world_aff,
                                                            planSeed.stateframe_ids,
                                                            planSeed.stateframe_values,
                                                            posture_goal_msg);
      string channel = "POSTURE_GOAL";
      self->lcm->publish(channel, &posture_goal_msg);
      }
      else if(! strcmp(name,PARAM_REACH_STARTING_POSE)){
      drc::robot_state_t pose_goal_msg;
      visualization_utils::decodeAndExtractFirstFrameInKeyframePlanFromStorage(T_world_aff,
                                                            planSeed.stateframe_ids,
                                                            planSeed.stateframe_values,
                                                           pose_goal_msg);
      string channel = "CANDIDATE_ROBOT_ENDPOSE";
      self->lcm->publish(channel, &pose_goal_msg);
      }

    }
    else if (! strcmp(name, PARAM_UNSTORE_PLAN)) {
      string selected_plan;
      selected_plan = self->_planseeds[bot_gtk_param_widget_get_enum(pw, PARAM_PLAN_SEED_LIST)];
      string otdf_models_path = string(getModelsPath()) + "/otdf/"; 
      string otdf_filepath,plan_xml_dirpath;
      otdf_filepath =  otdf_models_path + (it->second.otdf_type) +".otdf";
      plan_xml_dirpath=  otdf_models_path + "stored_plans/"; 
      cout << "Unstoring Plan : " <<selected_plan << endl;
      PlanSeed::unstoreFromOtdf(otdf_filepath,plan_xml_dirpath,selected_plan);
    }
    else if(!strcmp(name, PARAM_COMMIT_TO_COLLISION_SERVER) )
    {
      drc::workspace_object_urdf_t msg;
      msg.utime =bot_timestamp_now();
      msg.otdf_type =it->second.otdf_type;
      msg.uid =it->second.uid;
      msg.urdf_xml_string =otdf::convertObjectInstanceToURDFstring(it->second._otdf_instance);
      string channel = "COLLISION_AVOIDANCE_URDFS";
      self->lcm->publish(channel, &msg);
    }
    else if (! strcmp(name, PARAM_HALT_ALL_OPT)) {
      // emit global keyboard signal
      KDL::Frame T_geom_lhandpose = KDL::Frame::Identity();
      KDL::Frame T_geom_rhandpose = KDL::Frame::Identity();
      
      //T_geom_rhandpose = KDL::Frame::Identity();
      //T_geom_rhandpose.M =  KDL::Rotation::RPY((M_PI/4),0,(M_PI/4));
      //double x,y,z,w;
      //T_geom_rhandpose.M.GetQuaternion(x,y,z,w);
      //std::cout << w << " " << x <<" " << y << " " << z << std::endl;
      drc::grasp_opt_control_t msg;
   
      int grasp_type = msg.SANDIA_RIGHT;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;
      int contact_mask = bot_gtk_param_widget_get_enum (pw, PARAM_HAND_CONTACT_MASK_SELECT);    
      int drake_control =msg.HALT;//or NEW=0, RESET=1, HALT=2;
      int uid = 0;
      // Halt all active optimizations.
      std::vector<int> OptChannelIdList;
      std::vector<int> OptChannelHandUidList;
      std::string channel;
      self->graspOptStatusListener->getAllReservedOptChannelIdsandHandUids(OptChannelIdList,OptChannelHandUidList);
      for (unsigned int i=0;i<OptChannelIdList.size();i++)
      {
        std::stringstream oss;
        oss << "INIT_GRASP_OPT_" << OptChannelIdList[i];
        channel = oss.str();
        uid = OptChannelHandUidList[i];
        self->initGraspOptPublisher->publishGraspOptControlMsg(channel,T_geom_lhandpose,T_geom_rhandpose,grasp_type,contact_mask,drake_control,uid,1.0);
      }

    }
    else if(!strcmp(name,PARAM_OTDF_ADJUST_PARAM)) {
      self->instance_selection  = std::string(self->object_selection);
      spawn_adjust_params_popup(self);
    }
    else if(!strcmp(name,PARAM_OTDF_ADJUST_DOF)) {
      self->instance_selection  = std::string(self->object_selection);  
      spawn_adjust_dofs_popup(self);
    }
    else if(!strcmp(name,PARAM_GET_MANIP_PLAN)) {
      //cout << "publishes ee motion constraints for all associated sticky hands and feet \n";
      bool is_retractable = false;
      publish_EE_locii_and_get_manip_plan(self,is_retractable);
    }
    else if(!strcmp(name,PARAM_GET_RETRACTABLE_MANIP_PLAN)){
      bool is_retractable = true;
      publish_EE_locii_and_get_manip_plan(self,is_retractable);
    }
    else if(!strcmp(name,PARAM_SET_EE_CONSTRAINT)) {
        spawn_get_ee_constraint_popup(self);
    }
    else if(!strcmp(name,PARAM_GET_MANIP_MAP)) {
      self->instance_selection  = std::string(self->object_selection);  
      spawn_set_manip_map_dof_range_popup(self);
    }
    else if(!strcmp(name,PARAM_SEND_POSE_GOAL)){
      string channel = "POSE_GOAL";
      KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
      publish_pose_goal(self,channel,T_world_body_desired,false);  
    }
    else if(!strcmp(name,PARAM_SEND_POSE_GOAL2)){
      string channel = "POSE_GOAL";
      KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
      publish_pose_goal(self,channel,T_world_body_desired,true);  
    }    
    else if(!strcmp(name,PARAM_SEND_POSE_GOAL3)){
      string channel = "POSE_GOAL";
      KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
      publish_pose_goal_4_aff_motion(self,channel,T_world_body_desired);  
    }
    else if(!strcmp(name,PARAM_SELECT_MATE_AXIS_FOR_EE_TELEOP))
    {
      self->active_mate_axis =  bot_gtk_param_widget_get_enum(pw,PARAM_SELECT_MATE_AXIS_FOR_EE_TELEOP);
    } 
    else if(!strcmp(name,PARAM_SELECT_EE_TYPE))
    {
      self->active_ee = bot_gtk_param_widget_get_enum(pw,PARAM_SELECT_EE_TYPE);
    }
    else if(!strcmp(name,PARAM_ENGAGE_EE_TELEOP))
    {
      std::string ee_name;
      if(self->active_ee==drc::ee_teleop_transform_t::LEFT_HAND)
         ee_name = "left_palm";
      else if(self->active_ee==drc::ee_teleop_transform_t::RIGHT_HAND)
         ee_name = "right_palm";  
              
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.begin();
      while (hand_it!=self->stickyHandCollection->_hands.end()) 
      {
        std::string hand_name = std::string(hand_it->second.object_name);
        if ((hand_name == (it->first))&&(hand_it->second.is_melded))
        {

          
          int ee_type= hand_it->second.hand_type;   
          if(ee_type==0)
            self->active_ee=drc::ee_teleop_transform_t::LEFT_HAND;
          else if(ee_type==1)
            self->active_ee=drc::ee_teleop_transform_t::RIGHT_HAND;
          else{
            cerr<< "ERROR: unknown handtype" << ee_type 
            << ". To engage EE teleop the object must have a valid melded sticky hand\n";  
             return;
            }
            KDL::Frame T_world_mate_female_end;
           
            double r,p,y;
            string link_name;
            link_name= it->second._gl_object->_mate_start_link;
            it->second._gl_object->get_link_frame(link_name,T_world_mate_female_end);        
             T_world_mate_female_end.M.GetRPY(r,p,y);
            //cout << "link_name" << link_name << " " << r << " "<< p << " "<<y << endl;
                                       
            KDL::Vector worldframe_mateaxis,ux,uy,uz;
            ux[0]=1;ux[1]=0;ux[2]=0;
            uy[0]=0;uy[1]=1;uy[2]=0;
            uz[0]=0;uz[1]=0;uz[2]=1;
 
            if(self->active_mate_axis==0)
               worldframe_mateaxis = T_world_mate_female_end.M*ux;
            else if(self->active_mate_axis==1)
               worldframe_mateaxis = T_world_mate_female_end.M*uy;
            else if(self->active_mate_axis==2)
               worldframe_mateaxis = T_world_mate_female_end.M*uz;
            worldframe_mateaxis.Normalize();
            KDL::Frame T_world_palm,T_world_link,T_link_palm;
            link_name= it->second._gl_object->_mate_end_link;//hand_it->second.geometry_name;
            it->second._gl_object->get_link_frame(link_name,T_world_link);
            //KDL::Frame T_world_object = it->second._gl_object->_T_world_body; // 
            self->robotStateListener->_gl_robot->get_link_frame(ee_name,T_world_palm);
            T_link_palm = T_world_link.Inverse()*T_world_palm;   
            string channel = "PALM_TELEOP_TRANSFORM";
            publish_ee_transform_to_engage_ee_teleop(channel, self->active_ee,
            worldframe_mateaxis,T_link_palm, self); 
            return;  
          
        } 
        hand_it++;
      }// end while
    }
    else if(!strcmp(name,PARAM_MATE)){
        // get desired state from popup sliders
        KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
        map<string, double> jointpos_in=it->second._gl_object->_current_jointpos;
        typedef map<string,boost::shared_ptr<otdf::Joint> > joints_mapType;
        for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++) {     
            double dof_pos = 0;
            string token  = "mate::";
            string joint_name = joint->first;
            size_t found = joint_name.find(token);  
            if ((found!=std::string::npos)&&(joint->second->type!=(int) otdf::Joint::FIXED)) {
                dof_pos =  0;
                jointpos_in.find(joint->first)->second =  dof_pos; 
                //cout <<  joint->first << " DOF changed to " << dof_pos*(180/M_PI) << endl;
            }
        }
        if(!it->second._gl_object->is_future_state_changing()) {
          it->second._gl_object->set_future_state_changing(true); 
        }
        it->second._gl_object->set_future_state(T_world_object,jointpos_in);
  
      // set dependent bodies desired motion state to mate state
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.begin();
      while (hand_it!=self->stickyHandCollection->_hands.end()) 
      {
         std::string hand_name = std::string(hand_it->second.object_name);
         if (hand_name == (it->first))
         {
            
            KDL::Frame T_world_graspgeometry;
            it->second._gl_object->get_link_geometry_future_frame(hand_it->second.geometry_name,T_world_graspgeometry);
            //hand_it->second._gl_hand->log_motion_trail(true); // not accumulating states on draw (so why have this?)
            KDL::Frame T_accumulationFrame_body;
            //T_accumulationFrame_body = T_accumulationFrame_currentWorldFrame*_T_world_body;
            
            KDL::Frame T_world_object = it->second._gl_object->_T_world_body; // accumulate in current object frame not_T_world_body_future?
            KDL::Frame T_object_graspgeometry = T_world_object.Inverse()*T_world_graspgeometry;
            T_accumulationFrame_body = (T_object_graspgeometry)*hand_it->second._gl_hand->_T_world_body;//T_obj_geom*T_geom_hand
            hand_it->second._gl_hand->clear_desired_body_motion_history();
            hand_it->second._gl_hand->_desired_body_motion_history.push_back(T_accumulationFrame_body);
            //cout << "accumulating desired_motion_history" << hand_it->second._gl_hand->_desired_body_motion_history.size() << endl;
         }  
         hand_it++;
      }

      typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
      sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.begin();
      while (foot_it!=self->stickyFootCollection->_feet.end()) 
      {
         std::string foot_name = std::string(foot_it->second.object_name);
         if (foot_name == (it->first))
         {
            foot_it->second._gl_foot->clear_desired_body_motion_history();
            KDL::Frame T_world_geometry;
            it->second._gl_object->get_link_geometry_future_frame(hand_it->second.geometry_name,T_world_geometry);
            //foot_it->second._gl_hand->log_motion_trail(true);
            KDL::Frame T_accumulationFrame_body;
            //T_accumulationFrame_body = T_accumulationFrame_currentWorldFrame*_T_world_body;
           KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
            KDL::Frame T_object_geometry = T_world_object.Inverse()*T_world_geometry;
            T_accumulationFrame_body = (T_object_geometry)*foot_it->second._gl_foot->_T_world_body;
            foot_it->second._gl_foot->_desired_body_motion_history.push_back(T_accumulationFrame_body);
            
         }
         foot_it++;
      }
      bot_viewer_request_redraw(self->viewer);
        
        
    }
    
    bot_viewer_request_redraw(self->viewer);
    if(    strcmp(name, PARAM_FOOT_CONTACT_MASK_SELECT)
        && strcmp(name,PARAM_DIL_FACTOR)
        && strcmp(name, PARAM_HAND_CONTACT_MASK_SELECT)
        && strcmp(name, PARAM_ADJUST_DESIRED_DOFS_VIA_SLIDERS)
        && strcmp(name,PARAM_SELECT_MATE_AXIS_FOR_EE_TELEOP)
        && strcmp(name,PARAM_SELECT_EE_TYPE)
        && strcmp(name,PARAM_PLAN_SEED_LIST)
        && strcmp(name,PARAM_SELECT_FLIP_DIM)
        && strcmp(name, PARAM_RESET_DESIRED_STATE)
        && strcmp(name, PARAM_FLIP_GEOMETRY)
        && strcmp(name, PARAM_ADJUST_DESIRED_DOFS_VIA_SLIDERS)
        && strcmp(name, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT)
        && strcmp(name, PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT)     
        && strcmp(name, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT)
        && strcmp(name, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT)    
        && strcmp(name,PARAM_OTDF_ADJUST_PARAM)
        && strcmp(name,PARAM_OTDF_ADJUST_DOF)
        && strcmp(name,PARAM_SET_EE_CONSTRAINT)
        )
      gtk_widget_destroy(self->dblclk_popup); // destroy for every other change except mask selection
  }
  
  //--------------------------------------------------------------------------
  
  static void on_dblclk_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    self->dblclk_popup = NULL;
  }
  
  //--------------------------------------------------------------------------

  static void spawn_object_geometry_dblclk_popup (RendererAffordances *self)
  {
 
    bool has_seeds = otdf_instance_has_seeds(self,self->object_selection);
    if((self->marker_selection  == " "))
       set_hand_init_position(self); 
 
    GtkWidget *window, *close_button, *vbox;
    

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 150, 250);
    gint pos_x, pos_y;
    gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
    pos_x+=125;    pos_y-=75;
   // gint gdk_screen_height  (void);//Returns the height of the default screen in pixels.
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "dblclk");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    

    

    int num_hand_contact_masks =  2;
    char ** hand_contact_masks =(char **) calloc(num_hand_contact_masks, sizeof(char *));
    int* hand_contact_nums = (int *)calloc(num_hand_contact_masks, sizeof(int));
    hand_contact_masks[0]=(char*) "ALL"; hand_contact_masks[1]=(char*) "FINGERS";
    hand_contact_nums[0]=drc::grasp_opt_control_t::ALL; 
    hand_contact_nums[1]=drc::grasp_opt_control_t::FINGERS_ONLY;
    
    foot_contact_mask_type_t temp;
    int num_foot_contact_masks =  4;
    char ** foot_contact_masks =(char **) calloc(num_foot_contact_masks, sizeof(char *));
    int* foot_contact_nums = (int *)calloc(num_foot_contact_masks, sizeof(int));
    foot_contact_masks[0]=(char*) "Org"; foot_contact_nums[0]=visualization_utils::ORG; 
    foot_contact_masks[1]=(char*) "Heel";foot_contact_nums[1]=visualization_utils::HEEL;
    foot_contact_masks[2]=(char*) "Toe"; foot_contact_nums[2]=visualization_utils::TOE;
    foot_contact_masks[3]=(char*) "Mid"; foot_contact_nums[3]=visualization_utils::MID;
    
   
    /*if((!has_seeds)&&((self->marker_selection  == " ")
      ||self->otdf_instance_hold._gl_object->is_bodypose_adjustment_enabled()
      ||self->otdf_instance_hold._gl_object->is_jointdof_adjustment_enabled())) */
    //if((!has_seeds)&&((self->marker_selection  == " ")||self->selection_hold_on)) 
    
   

    
    BotGtkParamWidget *pw;
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    
    bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_DELETE, NULL);
        
    if(((self->marker_selection  == " ")||self->selection_hold_on)) 
    {
      //bot_gtk_param_widget_add_separator (pw,"Post-fitting adjust");
      bot_gtk_param_widget_add_separator (pw,"(of params/currentstate)");
      bot_gtk_param_widget_add_separator (pw,"(via markers/sliders)");
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_PARAM, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_DOF, NULL); 
      bot_gtk_param_widget_add_enum(pw, PARAM_SELECT_FLIP_DIM,
                                     BOT_GTK_PARAM_WIDGET_MENU,2,
                                     "X",0,
                                     "Y",1,
                                     "Z",2, NULL); 
      bot_gtk_param_widget_add_buttons(pw,PARAM_FLIP_GEOMETRY, NULL); 

      bool val=false;
      bool val2=false;
      if(self->selection_hold_on){
       val =  self->otdf_instance_hold._gl_object->is_bodypose_adjustment_enabled();
       val2 = self->otdf_instance_hold._gl_object->is_jointdof_adjustment_enabled();
      }
      bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT, val, NULL);
      bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT, val2, NULL);
    }
    
    BotGtkParamWidget *ee_seeds_pw;
    ee_seeds_pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    if((self->marker_selection  == " ")) 
    {
      //bot_gtk_param_widget_add_separator (ee_seeds_pw,"Seed Management");
      bot_gtk_param_widget_add_separator (ee_seeds_pw,"(contact filter)");
      bot_gtk_param_widget_add_enumv (ee_seeds_pw, PARAM_HAND_CONTACT_MASK_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                            drc::grasp_opt_control_t::ALL,
				                            num_hand_contact_masks,
			                              (const char **)  hand_contact_masks,
			                              hand_contact_nums);
			bot_gtk_param_widget_add_enumv (ee_seeds_pw, PARAM_FOOT_CONTACT_MASK_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                            visualization_utils::ORG,
				                            num_foot_contact_masks,
			                              (const char **)  foot_contact_masks,
			                              foot_contact_nums);                              
			bot_gtk_param_widget_add_separator (ee_seeds_pw,"(seed-opt control)");
			//spinbox for dil factor from -2 to 2 in 0.1 inc, 1 by default
		  bot_gtk_param_widget_add_double(ee_seeds_pw,PARAM_DIL_FACTOR, BOT_GTK_PARAM_WIDGET_SPINBOX,
                                                0.1, 2.0, 0.1, 1);			
      bot_gtk_param_widget_add_buttons(ee_seeds_pw,PARAM_SEED_LH, NULL);
      bot_gtk_param_widget_add_buttons(ee_seeds_pw,PARAM_SEED_RH, NULL);
      bot_gtk_param_widget_add_buttons(ee_seeds_pw,PARAM_SEED_LF, NULL);
      bot_gtk_param_widget_add_buttons(ee_seeds_pw,PARAM_SEED_RF, NULL);    
      bot_gtk_param_widget_add_buttons(ee_seeds_pw,PARAM_CLEAR_SEEDS, NULL);
      bot_gtk_param_widget_add_buttons(ee_seeds_pw,PARAM_HALT_ALL_OPT, NULL);
      bot_gtk_param_widget_add_buttons(ee_seeds_pw,PARAM_SET_EE_CONSTRAINT, NULL);    
      
    }
     BotGtkParamWidget *set_des_state_pw;
     set_des_state_pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
     
     BotGtkParamWidget *get_plan_pw;
     get_plan_pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new()); 
     BotGtkParamWidget *mating_pw;
     mating_pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
        
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it= self->affCollection->_objects.find(self->object_selection);
      if(it!=self->affCollection->_objects.end())
      {
       //has_seeds = true;
         if((has_seeds)&&(!self->selection_hold_on))  
         {
            bool val,val2;
            val = false;
            val2 = false;
            if(it!=self->affCollection->_objects.end()){
            val = it->second._gl_object->is_bodypose_adjustment_enabled();
            if(!val)
             val2 = it->second._gl_object->is_jointdof_adjustment_enabled();
            }   
            
            //bot_gtk_param_widget_add_separator (set_des_state_pw,"Set desired state");
            bot_gtk_param_widget_add_separator (set_des_state_pw,"(via markers/sliders)");

           
            bot_gtk_param_widget_add_booleans(set_des_state_pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT, val, NULL);
            bot_gtk_param_widget_add_booleans(set_des_state_pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT, val2, NULL);
            bot_gtk_param_widget_add_buttons(set_des_state_pw, PARAM_ADJUST_DESIRED_DOFS_VIA_SLIDERS,NULL);
            bot_gtk_param_widget_add_buttons(set_des_state_pw, PARAM_RESET_DESIRED_STATE,NULL);
            
            //bot_gtk_param_widget_add_separator (get_plan_pw,"Get Manip Plan/Map");
            bot_gtk_param_widget_add_separator (get_plan_pw,"(via EE pt/motion/range goal)");     
            bot_gtk_param_widget_add_separator (get_plan_pw,"(for approval)");
            bot_gtk_param_widget_add_buttons(get_plan_pw,PARAM_GET_MANIP_PLAN, NULL);
            bot_gtk_param_widget_add_buttons(get_plan_pw,PARAM_GET_RETRACTABLE_MANIP_PLAN, NULL);
            bot_gtk_param_widget_add_buttons(get_plan_pw,PARAM_GET_MANIP_MAP,NULL);
            bot_gtk_param_widget_add_buttons(get_plan_pw,PARAM_SEND_POSE_GOAL,NULL);
            //if(it->second._gl_object->is_future_display_active())
            bot_gtk_param_widget_add_buttons(get_plan_pw,PARAM_SEND_POSE_GOAL2,NULL);
            bot_gtk_param_widget_add_buttons(get_plan_pw,PARAM_SEND_POSE_GOAL3,NULL);
         }
      
          bot_gtk_param_widget_add_buttons(mating_pw,PARAM_MATE, NULL);

          // If affordance is mateable SHOW ee teleop settings.
          
          //To engage EE teleop the object must have a valid melded sticky hand, and it must be melded to robot state as well
          if((it->second._gl_object->is_mateable())&&(it->second.is_melded) ) {
            //bool has_melded_seeds = object_has_melded_sticky_hands(it->first);
              bot_gtk_param_widget_add_separator (mating_pw,"EE Teleop Settings");
                /*bot_gtk_param_widget_add_enum(pw, PARAM_SELECT_EE_TYPE, 
                                     BOT_GTK_PARAM_WIDGET_MENU,self->active_ee, 
                                    "Left hand", drc::ee_teleop_transform_t::LEFT_HAND,
                                    "Right Hand", drc::ee_teleop_transform_t::RIGHT_HAND, NULL);*/
                bot_gtk_param_widget_add_enum(mating_pw, PARAM_SELECT_MATE_AXIS_FOR_EE_TELEOP,
                                     BOT_GTK_PARAM_WIDGET_MENU,self->active_mate_axis,
                                     "MATE::X",0,
                                     "MATE::Y",1,
                                     "MATE::Z",2, NULL); 
                bot_gtk_param_widget_add_buttons(mating_pw,PARAM_ENGAGE_EE_TELEOP, NULL);
          }
      
      }// end if
 
 
    BotGtkParamWidget *planseed_pw;
    planseed_pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
 
    //bot_gtk_param_widget_add_separator (planseed_pw,"Plan Seed Management");
    bot_gtk_param_widget_add_buttons(planseed_pw,PARAM_STORE_PLAN, NULL); 
   
    if((self->marker_selection  == " ")&&(it!=self->affCollection->_objects.end())) 
    {
      self->_planseeds = vector<string>();
      std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
      std::string otdf_filepath;
      otdf_filepath =  otdf_models_path + (it->second.otdf_type) +".otdf";
      PlanSeed::getList(otdf_filepath,self->_planseeds);
      if(self->_planseeds.size()>0)
      {
        vector<string> _names;
        vector<const char*> seed_names;
        vector<int> seed_nums;
        for(int i = 0; i < self->_planseeds.size(); ++i)
        {
           string token  = "::";
           size_t found = self->_planseeds[i].find(token);  
           _names.push_back(self->_planseeds[i].substr(found+token.size()));
           seed_names.push_back(_names[i].c_str());
           seed_nums.push_back(i);
        }
        bot_gtk_param_widget_add_enumv (planseed_pw, PARAM_PLAN_SEED_LIST, BOT_GTK_PARAM_WIDGET_MENU, 
                                        0,
                                        self->_planseeds.size(),
                                        &seed_names[0],
                                        &seed_nums[0]);
        bot_gtk_param_widget_add_buttons(planseed_pw,PARAM_REACH_STARTING_POSTURE, NULL);
        bot_gtk_param_widget_add_buttons(planseed_pw,PARAM_REACH_STARTING_POSE, NULL);
        bot_gtk_param_widget_add_buttons(planseed_pw,PARAM_LOAD_PLAN, NULL);
        bot_gtk_param_widget_add_buttons(planseed_pw,PARAM_UNSTORE_PLAN, NULL); 
      }
    }
    
   
    
    BotGtkParamWidget *col_server_pw;
    col_server_pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    bot_gtk_param_widget_add_buttons(col_server_pw,PARAM_COMMIT_TO_COLLISION_SERVER, NULL);     
    

    //cout <<self->selection << endl; // otdf_type::geom_name
    GtkWidget * param_adjust_pane =  gtk_expander_new("Post-fitting adjust");
    gtk_container_add (GTK_CONTAINER (param_adjust_pane), GTK_WIDGET( pw));    
    gtk_expander_set_expanded(GTK_EXPANDER(param_adjust_pane),(gboolean) TRUE);
    
    GtkWidget * ee_seeds_pane =  gtk_expander_new("Seed Management");
    gtk_container_add (GTK_CONTAINER (ee_seeds_pane), GTK_WIDGET( ee_seeds_pw));
    gtk_expander_set_expanded(GTK_EXPANDER(ee_seeds_pane),(gboolean) FALSE);
    
    GtkWidget * set_des_state_pane =  gtk_expander_new("Set Desired State");
    gtk_container_add (GTK_CONTAINER (set_des_state_pane), GTK_WIDGET( set_des_state_pw));   
    gtk_expander_set_expanded(GTK_EXPANDER(set_des_state_pane),(gboolean) TRUE);
    
    GtkWidget * get_plan_pane =  gtk_expander_new("Get Manip Plan/Map");
    gtk_container_add (GTK_CONTAINER (get_plan_pane), GTK_WIDGET( get_plan_pw));
    gtk_expander_set_expanded(GTK_EXPANDER(get_plan_pane),(gboolean) TRUE);
    
    GtkWidget * mating_pane =  gtk_expander_new("Mating");
    gtk_container_add (GTK_CONTAINER (mating_pane), GTK_WIDGET( mating_pw));
    GtkWidget * planseed_pane =  gtk_expander_new("Plan Seed Management");
    gtk_container_add (GTK_CONTAINER (planseed_pane), GTK_WIDGET( planseed_pw));
    gtk_expander_set_expanded(GTK_EXPANDER(planseed_pane),(gboolean) FALSE);
    
    GtkWidget *col_server_pane =  gtk_expander_new("collision_server_comms");
    gtk_container_add (GTK_CONTAINER (col_server_pane), GTK_WIDGET(col_server_pw));
    gtk_expander_set_expanded(GTK_EXPANDER(col_server_pane),(gboolean) FALSE);
    
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);
    g_signal_connect(G_OBJECT(ee_seeds_pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);
    g_signal_connect(G_OBJECT(set_des_state_pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);
    g_signal_connect(G_OBJECT(get_plan_pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);  
    g_signal_connect(G_OBJECT(mating_pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);      
    g_signal_connect(G_OBJECT(planseed_pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);
    g_signal_connect(G_OBJECT(col_server_pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);
    
    self->dblclk_popup  = window;

    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),"clicked",G_CALLBACK (on_popup_close),(gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy", G_CALLBACK(on_dblclk_popup_close), self); 


    vbox = gtk_vbox_new (FALSE, 3);
    
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
  
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(param_adjust_pane), FALSE, FALSE, 5);
    if((self->marker_selection  == " ")) 
      gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(ee_seeds_pane), FALSE, FALSE, 5);
    if(it!=self->affCollection->_objects.end())
    {
      //has_seeds = true;
      if((has_seeds)&&(!self->selection_hold_on))  
      {
        gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(set_des_state_pane), FALSE, FALSE, 5);
        gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(get_plan_pane), FALSE, FALSE, 5);  
      }
      gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(mating_pane), FALSE, FALSE, 5);
    }
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(planseed_pane), FALSE, FALSE, 5);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(col_server_pane), FALSE, FALSE, 5);

    gtk_widget_show_all(window); 

 
      free(hand_contact_masks);
      free(hand_contact_nums);
      free(foot_contact_masks);
      free(foot_contact_nums);
  }
  
}
#endif
