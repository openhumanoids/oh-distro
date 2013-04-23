#ifndef OBJECT_INTERACTION_GUI_UTILS_HPP
#define OBJECT_INTERACTION_GUI_UTILS_HPP

#define PARAM_SEED_LH "Seed LHand"
#define PARAM_SEED_RH "Seed RHand"
#define PARAM_SEED_LF "Seed LFoot"
#define PARAM_SEED_RF "Seed RFoot"
#define PARAM_CLEAR_SEEDS "Clear Seeds"

#define PARAM_HALT_ALL_OPT "Halt All Opts"

#define PARAM_PREGRASP "Reach (Pre-grasp)" 
#define PARAM_PALM_TOUCH "Palm Touch"   // publishes pre-grasp pose as ee_goal for reaching controller
#define PARAM_GET_CLOSE "Get Close" 
#define PARAM_COMMIT   "Commit Grasp"  // commits grasp state as setpoint and enables grasp controller
#define PARAM_EXECUTE  "Execute Grasp" // publishes grasp pose as ee_goal for reaching controller. Simultaneously grasp controller executes only if ee pose is close to the committed grasp pose (if inFunnel, execute grasp)
#define PARAM_EE_MOTION "Move EE"


#define PARAM_RESEED "Re-seed"
#define PARAM_HALT_OPT "Halt Opt"
#define PARAM_DELETE "Delete"
#define PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT "Adjust Cur BodyPose"
#define PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT "Adjust Cur JointDofs"
#define PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT "Set Des BodyPose"
#define PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT "Set Des JointDofs"
#define PARAM_ADJUST_DOFS_VIA_SLIDERS "Adjust (via Sliders)"
#define PARAM_RESET_DESIRED_STATE "Reset"
#define PARAM_CONTACT_MASK_SELECT "Mask"

#include "renderer_affordances.hpp"
#include "otdf_instance_management_gui_utils.hpp"

using namespace renderer_affordances;

namespace renderer_affordances_gui_utils
{
//--------------------------------------------------------------------------------
//  OTDF object dblclk popup and associated cbs.


//---------------------------------------------------------------
// SECOND STAGE ADJUST DOFS POPUP
  static void on_adjust_dofs_popup_close2 (BotGtkParamWidget *pw, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    std::string instance_name=  self->object_selection;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    it->second._gl_object->set_future_state_changing(false);
    self->second_stage_popup  = NULL;
  //   TODO: Send publish desired affordance state command msg    
  
  }

  static void on_otdf_adjust_dofs_widget_changed2(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    std::string instance_name=  self->object_selection;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    KDL::Frame T_world_object = it->second._gl_object->_T_world_body;

      
    if(!it->second._gl_object->is_future_state_changing()) {
       std::map<std::string, double> jointpos_in;
      jointpos_in = it->second._gl_object->_future_jointpos;
    
      it->second._gl_object->set_future_state(T_world_object,jointpos_in);
      it->second._gl_object->set_future_state_changing(true);

     // clear previously accumulated motion states for all dependent bodies
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
      while (hand_it!=self->sticky_hands.end()) 
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
        desired_roll   =  bot_gtk_param_widget_get_double (pw, "roll");
        desired_pitch   =  bot_gtk_param_widget_get_double (pw, "pitch");
        desired_yaw   =  bot_gtk_param_widget_get_double (pw, "yaw");  
        T_world_object.M = KDL::Rotation::RPY(desired_roll,desired_pitch,desired_yaw);
    }
    
    
    std::map<std::string, double> jointpos_in;
      
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double desired_dof_pos = 0; // get current desired dof pos.
      if(joint->second->type!=(int) otdf::Joint::FIXED) {
          desired_dof_pos =  bot_gtk_param_widget_get_double (pw, joint->first.c_str());
          jointpos_in.insert(make_pair(joint->first, desired_dof_pos)); 
       cout <<  joint->first << " dof changed to " << desired_dof_pos << endl;
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
    gtk_window_set_title(GTK_WINDOW(window), "Adjust Dofs");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    std::string instance_name=  self->object_selection;

    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    
    KDL::Frame T_world_object = it->second._gl_object->_T_world_body_future;
    double current_roll,current_pitch, current_yaw;
    T_world_object.M.GetRPY(current_roll,current_pitch,current_yaw);
    // check root link
    // if root link is not world
    // add x,y,z,roll,pitch,yaw sliders.
    if(it->second._otdf_instance->root_link_->name!= "world")// object is not bolted to the world.
    {
        bot_gtk_param_widget_add_double(pw, "x", BOT_GTK_PARAM_WIDGET_SLIDER, -4, 4, .01, T_world_object.p[0]); 
        bot_gtk_param_widget_add_double(pw, "y", BOT_GTK_PARAM_WIDGET_SLIDER, -4, 4, .01, T_world_object.p[1]);
        bot_gtk_param_widget_add_double(pw, "z", BOT_GTK_PARAM_WIDGET_SLIDER, -4, 4, .01, T_world_object.p[2]);
        bot_gtk_param_widget_add_double(pw, "roll", BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI, 2*M_PI, .01, current_roll); 
        bot_gtk_param_widget_add_double(pw, "pitch", BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI, 2*M_PI, .01, current_pitch); 
        bot_gtk_param_widget_add_double(pw, "yaw", BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI, 2*M_PI, .01, current_yaw); 
    }

    // Need tracked joint positions of all objects.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double current_dof_position = 0;// TODO: dof pos tracking
      if(joint->second->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
        if(joint->second->type==(int) otdf::Joint::CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI, 2*M_PI, .01, current_dof_position); 
        }
        else{
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          joint->second->limits->lower, joint->second->limits->upper, .01, current_dof_position);
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
          -2*M_PI, 2*M_PI, .01, current_dof_position); 
          }
          else{
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          jp_it->second->joint_set[i]->limits->lower, jp_it->second->joint_set[i]->limits->upper, .01, current_dof_position);
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
    object_instance_map_type_::iterator it= self->instantiated_objects.find(self->object_selection);

  
    if (! strcmp(name, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT)) {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT);
      if(val){
        bot_gtk_param_widget_set_bool(pw, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT,false); 
        std::cout << "enabling bodypose adjustment for object " <<self->object_selection << std::endl;
      }
     else{
         std::cout << "disabling bodypose adjustment for object " <<self->object_selection << std::endl;
         bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT);
         if((self->marker_selection  != " ")&&(!val)) {
          if(it->second._gl_object->is_future_state_changing())
            it->second._gl_object->set_future_state_changing(false);   // when both jointdof and bodypose are disabled.
         }
      }  
        
      if(it!=self->instantiated_objects.end()){
        it->second._gl_object->enable_bodypose_adjustment(val);
        it->second._gl_object->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::THREE_D);
        it->second._gl_object->enable_jointdof_adjustment(false);    
      }
    }
    else if (! strcmp(name, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT)) {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT);
      if(val){
        bot_gtk_param_widget_set_bool(pw, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT,false);
        std::cout << "enabling jointdof adjustment for object " <<self->object_selection << " to "<< val << std::endl;
      }
      else{
          std::cout << "disabling jointdof adjustment for object " <<self->object_selection << std::endl;
         bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT);
         if((self->marker_selection  != " ")&&(!val)) {
          if(it->second._gl_object->is_future_state_changing())
            it->second._gl_object->set_future_state_changing(false);   // clear future state when both jointdof and bodypose are disabled.
         }
      }
      if(it!=self->instantiated_objects.end()){
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
          cout << "ok" << endl;
          self->otdf_instance_hold.otdf_type = it->second.otdf_type;
          //self->otdf_instance_hold.otdf_type = new string((*it->second.otdf_type));    // SEGFAULTS With Strings  .copy((char *)it->second.otdf_type.c_str(),it->second.otdf_type.length())
          cout << "ok" << endl;
          self->otdf_instance_hold._otdf_instance = otdf::duplicateOTDFInstance(it->second._otdf_instance);
          self->otdf_instance_hold._gl_object.reset();
          self->otdf_instance_hold._collision_detector.reset();
          self->otdf_instance_hold._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());     
          self->otdf_instance_hold._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(self->otdf_instance_hold._otdf_instance,self->otdf_instance_hold._collision_detector,true,"otdf_instance_hold"));
          self->otdf_instance_hold._otdf_instance->update();
          self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
          self->selection_hold_on=true;
        }
        
      }
     else{
        std::cout << "disabling bodypose adjustment for object " <<self->object_selection << std::endl;
        self->selection_hold_on=false;
      }  
      if(it!=self->instantiated_objects.end()){
        self->otdf_instance_hold._gl_object->enable_bodypose_adjustment(val);
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
          self->otdf_instance_hold._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());     
          self->otdf_instance_hold._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(self->otdf_instance_hold._otdf_instance,self->otdf_instance_hold._collision_detector,true,"otdf_instance_hold"));
          self->otdf_instance_hold._otdf_instance->update();
          self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
          self->selection_hold_on=true;
        }
        
      }
      else{
          std::cout << "disabling jointdof adjustment for object " <<self->object_selection << std::endl;
          self->selection_hold_on=false;
      }
      
      if(it!=self->instantiated_objects.end()){
         self->otdf_instance_hold._gl_object->enable_bodypose_adjustment(false); 
         self->otdf_instance_hold._gl_object->enable_jointdof_adjustment(val);  
       } 

    } 
    else if(! strcmp(name, PARAM_RESET_DESIRED_STATE)) {
       it->second._gl_object->set_future_state(it->second._gl_object->_T_world_body,it->second._gl_object->_current_jointpos);    
       bot_viewer_request_redraw(self->viewer);
    }       
    else if (! strcmp(name, PARAM_SEED_LH)) {
      drc::grasp_opt_control_t msg;
      int grasp_type = msg.SANDIA_LEFT;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;

      KDL::Frame T_geom_lhandpose = self->T_graspgeometry_lhandinitpos;
      KDL::Frame T_geom_rhandpose = KDL::Frame::Identity(); 

      int contact_mask = bot_gtk_param_widget_get_enum (pw, PARAM_CONTACT_MASK_SELECT);  
      int drake_control =msg.NEW;//or NEW=0, RESET=1, HALT=2;//       
      self->free_running_sticky_hand_cnt++;
      int uid = self->free_running_sticky_hand_cnt;
      std::string channel;
      if(self->graspOptStatusListener->isOptPoolReady())
       {
          int id =  self->graspOptStatusListener->getNextAvailableOptChannelId();
         
          if((id!=-1)&&(self->graspOptStatusListener->reserveOptChannel(id,uid)))
          {
           //int id =  1;
            std::stringstream oss;
            oss << "INIT_GRASP_SEED_OPT_" << id; 
            channel = oss.str();
            std::cout << channel << "  id :" << id << std::endl;
            self->initGraspOptPublisher->publishGraspOptControlMsg(channel,T_geom_lhandpose,T_geom_rhandpose,grasp_type,contact_mask,drake_control,uid);
          }             
      }
 
    }
    else if (! strcmp(name, PARAM_SEED_RH)) {
      drc::grasp_opt_control_t msg;
      int grasp_type = msg.SANDIA_RIGHT;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;

      KDL::Frame T_geom_lhandpose = KDL::Frame::Identity();
      KDL::Frame T_geom_rhandpose = self->T_graspgeometry_rhandinitpos;
      
      //T_geom_rhandpose = KDL::Frame::Identity();
      //T_geom_rhandpose.M =  KDL::Rotation::RPY((M_PI/4),0,(M_PI/4));
      //double x,y,z,w;
      //T_geom_rhandpose.M.GetQuaternion(x,y,z,w);
      //std::cout << w << " " << x <<" " << y << " " << z << std::endl;

      int contact_mask = bot_gtk_param_widget_get_enum (pw, PARAM_CONTACT_MASK_SELECT);  
      int drake_control =msg.NEW;//or NEW=0, RESET=1, HALT=2;
      self->free_running_sticky_hand_cnt++;
      int uid = self->free_running_sticky_hand_cnt;
       std::string channel;
      if(self->graspOptStatusListener->isOptPoolReady())
       {
          int id =  self->graspOptStatusListener->getNextAvailableOptChannelId();
          if((id!=-1)&&(self->graspOptStatusListener->reserveOptChannel(id,uid)))
          {
           //int id =  1;
            std::stringstream oss;
            oss << "INIT_GRASP_SEED_OPT_" << id; 
            channel = oss.str();
            std::cout << channel << "  id :" << id << std::endl;
            self->initGraspOptPublisher->publishGraspOptControlMsg(channel,T_geom_lhandpose,T_geom_rhandpose,grasp_type,contact_mask,drake_control,uid);
           }             
       }
    }
    else if (! strcmp(name, PARAM_SEED_LF)) {
      self->free_running_sticky_foot_cnt++;
      int uid = self->free_running_sticky_foot_cnt;
      int foot_type = 0;
      std::string object_name =self->object_selection;
      
      std::string object_geometry_name = self->link_selection;
      std::string object_name_token  = object_name + "_";
      size_t found = object_geometry_name.find(object_name_token);  
      std::string geometry_name =object_geometry_name.substr(found+object_name_token.size()); 
    
      Eigen::Vector3f eVx,eVy,eVz,diff;
      diff=self->ray_hit_drag-self->ray_hit;
      cout << diff.norm() << endl;
      if(diff.norm()< 0.001){
        diff << 0,0,1;
       } 
      diff.normalize();
      eVz = self->ray_hit_normal; eVz.normalize();
      eVy = eVz.cross(diff);  eVy.normalize();
      eVx = eVy.cross(eVz); eVx.normalize();

      KDL::Vector Vx,Vy,Vz;
      Vx[0]= eVx[0];Vx[1]= eVx[1];Vx[2]= eVx[2];
      Vy[0]= eVy[0];Vy[1]= eVy[1];Vy[2]= eVy[2];
      Vz[0]= eVz[0];Vz[1]= eVz[1];Vz[2]= eVz[2];

      KDL::Frame T_world_footcontact= KDL::Frame::Identity();
      T_world_footcontact.p[0] = self->ray_hit[0];
      T_world_footcontact.p[1] = self->ray_hit[1];
      T_world_footcontact.p[2] = self->ray_hit[2];
      KDL::Rotation tempM(Vx,Vy,Vz);
      T_world_footcontact.M = tempM;

      KDL::Frame T_world_objectgeometry = KDL::Frame::Identity(); 
      if(! it->second._gl_object->get_link_geometry_frame(geometry_name,T_world_objectgeometry))
      {
       cerr << " ERROR: failed to retrieve " << geometry_name<<" in object " << object_name <<endl;
      }
      KDL::Frame  T_objectgeometry_footcontact = (T_world_objectgeometry.Inverse())*T_world_footcontact;
      KDL::Frame T_objectgeometry_foot,T_contactframe_footframe;
      T_contactframe_footframe=self->candidateFootStepSeedManager->_T_groundframe_bodyframe_left;
      T_objectgeometry_foot=T_objectgeometry_footcontact*(T_contactframe_footframe);
    
      std::vector<std::string> joint_names;
      std::vector<double> joint_positions;
      joint_names.push_back("l_leg_uay");
      joint_names.push_back("l_leg_lax");
      joint_positions.push_back(0);
      joint_positions.push_back(0);      
      self->candidateFootStepSeedManager->add_or_update_sticky_foot(uid,foot_type,object_name,geometry_name, T_objectgeometry_foot,joint_names,joint_positions);	

    }
    else if (! strcmp(name, PARAM_SEED_RF)) {
      self->free_running_sticky_foot_cnt++;
      int uid = self->free_running_sticky_foot_cnt;
      int foot_type = 1;
      std::string object_name =self->object_selection;
      std::string object_geometry_name = self->link_selection;
      std::string object_name_token  = object_name + "_";
      size_t found = object_geometry_name.find(object_name_token);  
      std::string geometry_name =object_geometry_name.substr(found+object_name_token.size());
      
           Eigen::Vector3f eVx,eVy,eVz,diff;
      diff=self->ray_hit_drag-self->ray_hit;
      cout << diff.norm() << endl;
      if(diff.norm()< 0.001){
        diff << 0,0,1;
       } 
      diff.normalize();
      eVz = self->ray_hit_normal; eVz.normalize();
      eVy = eVz.cross(diff);  eVy.normalize();
      eVx = eVy.cross(eVz); eVx.normalize();

      KDL::Vector Vx,Vy,Vz;
      Vx[0]= eVx[0];Vx[1]= eVx[1];Vx[2]= eVx[2];
      Vy[0]= eVy[0];Vy[1]= eVy[1];Vy[2]= eVy[2];
      Vz[0]= eVz[0];Vz[1]= eVz[1];Vz[2]= eVz[2];

      KDL::Frame T_world_footcontact= KDL::Frame::Identity();
      T_world_footcontact.p[0] = self->ray_hit[0];
      T_world_footcontact.p[1] = self->ray_hit[1];
      T_world_footcontact.p[2] = self->ray_hit[2];
      KDL::Rotation tempM(Vx,Vy,Vz);
      T_world_footcontact.M = tempM;

      KDL::Frame T_world_objectgeometry = KDL::Frame::Identity(); 
      if(! it->second._gl_object->get_link_geometry_frame(geometry_name,T_world_objectgeometry))
      {
       cerr << " ERROR: failed to retrieve " << geometry_name<<" in object " << object_name <<endl;
      }
      KDL::Frame  T_objectgeometry_footcontact = (T_world_objectgeometry.Inverse())*T_world_footcontact;
      KDL::Frame T_objectgeometry_foot,T_contactframe_footframe;
      T_contactframe_footframe=self->candidateFootStepSeedManager->_T_groundframe_bodyframe_right;
      T_objectgeometry_foot=T_objectgeometry_footcontact*(T_contactframe_footframe);
      
      std::vector<std::string> joint_names;
      std::vector<double> joint_positions;
      joint_names.push_back("r_leg_uay");
      joint_names.push_back("r_leg_lax");
      joint_positions.push_back(0);
      joint_positions.push_back(0); 
      // Query Normal at point of dbl click. Drag direction gives foot direction.
      // Foot frame Z direction should point towards normal.     
      self->candidateFootStepSeedManager->add_or_update_sticky_foot(uid,foot_type,object_name,geometry_name, T_objectgeometry_foot,joint_names,joint_positions);	

    }
    else if (! strcmp(name, PARAM_CLEAR_SEEDS)) {
    
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
      while (hand_it!=self->sticky_hands.end()) 
      {
         if (hand_it->second.object_name == self->object_selection)
         {
            if(self->stickyhand_selection==hand_it->first)
               self->stickyhand_selection = " ";
            self->sticky_hands.erase(hand_it++);
         }
         else
            hand_it++;
      } 
      
      typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
      sticky_feet_map_type_::iterator foot_it = self->sticky_feet.begin();
      while (foot_it!=self->sticky_feet.end()) 
      {
         if (foot_it->second.object_name == self->object_selection)
         {
            if(self->stickyfoot_selection==foot_it->first)
               self->stickyfoot_selection = " ";
            self->sticky_feet.erase(foot_it++);
         }
         else
            foot_it++;
      } 
      
      self->selection_hold_on = false;
    
    }
    else if (! strcmp(name, PARAM_ADJUST_DOFS_VIA_SLIDERS)) {
        spawn_adjust_dofs_popup_2(self);
    }
    else if (! strcmp(name, PARAM_HALT_ALL_OPT)) {
      KDL::Frame T_geom_lhandpose = KDL::Frame::Identity();
      KDL::Frame T_geom_rhandpose = KDL::Frame::Identity();
      
      //T_geom_rhandpose = KDL::Frame::Identity();
      //T_geom_rhandpose.M =  KDL::Rotation::RPY((M_PI/4),0,(M_PI/4));
      //double x,y,z,w;
      //T_geom_rhandpose.M.GetQuaternion(x,y,z,w);
      //std::cout << w << " " << x <<" " << y << " " << z << std::endl;
      drc::grasp_opt_control_t msg;
   
      int grasp_type = msg.SANDIA_RIGHT;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;
      int contact_mask = bot_gtk_param_widget_get_enum (pw, PARAM_CONTACT_MASK_SELECT);    
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
        oss << "INIT_GRASP_SEED_OPT_" << OptChannelIdList[i];
        channel = oss.str();
        uid = OptChannelHandUidList[i];
        self->initGraspOptPublisher->publishGraspOptControlMsg(channel,T_geom_lhandpose,T_geom_rhandpose,grasp_type,contact_mask,drake_control,uid);
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
        
    bot_viewer_request_redraw(self->viewer);
    if(strcmp(name, PARAM_CONTACT_MASK_SELECT)&&strcmp(name, PARAM_ADJUST_DOFS_VIA_SLIDERS))
      gtk_widget_destroy(self->dblclk_popup); // destroy for every other change except mask selection
  }
  
  
  
  static void on_dblclk_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    // TODO: Send publish affordance command msg
    self->dblclk_popup = NULL;
  }
  


  static void spawn_object_geometry_dblclk_popup (RendererAffordances *self)
  {

    bool has_seeds = otdf_instance_has_seeds(self,self->object_selection);
    if((self->marker_selection  == " "))
       set_hand_init_position(self); 
       
    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw;

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
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    
    drc::grasp_opt_control_t msg;
    int num_masks =  2;
    char ** contact_masks =(char **) calloc(num_masks, sizeof(char *));
    int* contact_nums = (int *)calloc(num_masks, sizeof(int));
    contact_masks[0]=(char*) "ALL"; contact_masks[1]=(char*) "FINGERS";
    contact_nums[0]=msg.ALL; contact_nums[1]=msg.FINGERS_ONLY;
    
    
    if((!has_seeds)&&((self->marker_selection  == " ")
      ||self->otdf_instance_hold._gl_object->is_bodypose_adjustment_enabled()
      ||self->otdf_instance_hold._gl_object->is_jointdof_adjustment_enabled())) 
    {
      bot_gtk_param_widget_add_separator (pw,"Post-fitting adjust");
      bot_gtk_param_widget_add_separator (pw,"of params/curr state");
      bot_gtk_param_widget_add_separator (pw,"(via markers/sliders)");
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_PARAM, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_DOF, NULL); 

      bool val=false;
      bool val2=false;
      if(self->selection_hold_on){
       val =  self->otdf_instance_hold._gl_object->is_bodypose_adjustment_enabled();
       val2 = self->otdf_instance_hold._gl_object->is_jointdof_adjustment_enabled();
      }
      bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT, val, NULL);
      bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_CURRENT_JOINTDOF_ADJUSTMENT, val2, NULL);
    }
    
    if((self->marker_selection  == " ")) 
    {
    
      
      
      bot_gtk_param_widget_add_separator (pw,"Contact filter");
      bot_gtk_param_widget_add_enumv (pw, PARAM_CONTACT_MASK_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                            msg.ALL,
				                            num_masks,
			                              (const char **)  contact_masks,
			                              contact_nums);
		
      bot_gtk_param_widget_add_separator (pw,"Seed-opt control");
      bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_LH, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_RH, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_LF, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_RF, NULL);    
      bot_gtk_param_widget_add_buttons(pw,PARAM_CLEAR_SEEDS, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_HALT_ALL_OPT, NULL);

    
    }


   has_seeds = true;// turn on everything for now: just for testing. TODO: Remove later  
   if((has_seeds)&&(!self->selection_hold_on))  
   {
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it= self->instantiated_objects.find(self->object_selection);
      bool val,val2;
      val = false;
      val2 = false;
      if(it!=self->instantiated_objects.end()){
      val = it->second._gl_object->is_bodypose_adjustment_enabled();
      if(!val)
       val2 = it->second._gl_object->is_jointdof_adjustment_enabled();
      }   
      
      bot_gtk_param_widget_add_separator (pw,"Set desired state");
      bot_gtk_param_widget_add_separator (pw,"(via markers/sliders)");
      bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_DESIRED_BODYPOSE_ADJUSTMENT, val, NULL);
      bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_DESIRED_JOINTDOF_ADJUSTMENT, val2, NULL);
      bot_gtk_param_widget_add_buttons(pw, PARAM_ADJUST_DOFS_VIA_SLIDERS,NULL);
      bot_gtk_param_widget_add_buttons(pw, PARAM_RESET_DESIRED_STATE,NULL);
   }
    
    //cout <<self->selection << endl; // otdf_type::geom_name
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_object_geometry_dblclk_popup_param_widget_changed), self);

    self->dblclk_popup  = window;

    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),"clicked",G_CALLBACK (on_popup_close),(gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy", G_CALLBACK(on_dblclk_popup_close), self); 


    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
      gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 
 
 
      free(contact_masks);
      free(contact_nums);
    
  }
  
 
  
  //--------------------------------------------------------------------------
  // Sticky Hand Interaction
  //
  static void publish_eegoal_to_sticky_hand(boost::shared_ptr<lcm::LCM> &_lcm, StickyHandStruc &sticky_hand_struc,std::string ee_name, std::string channel,KDL::Frame &T_world_geometry,bool pregrasp_flag)
  {
    drc::ee_goal_t goalmsg;
    goalmsg.robot_name = "atlas";
    goalmsg.root_name = "pelvis";
    goalmsg.ee_name = ee_name;

    // desired ee position in world frame
    KDL::Frame T_world_ee,T_body_ee;
    
// Must account for the mismatch between l_hand and base in sandia_hand urdf. Publish in palm frame.   
   KDL::Frame  T_geometry_hand = sticky_hand_struc.T_geometry_hand;  // this is actually in a base frame that is not l_hand/r_hand.
//    KDL::Frame T_base_palm = KDL::Frame::Identity();
//    // this was there in urdf to make sure fingers are pointing in z axis.
//    T_base_palm.M =  KDL::Rotation::RPY(0,-(M_PI/2),0); 
//    KDL::Frame  T_geometry_palm = T_geometry_base*T_base_palm.Inverse()

    KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
    if(!sticky_hand_struc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
      cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
      
    T_world_ee = T_world_geometry*T_geometry_palm;
             
   if(pregrasp_flag)
   {
    KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; //this should be T_palm_base    
    KDL::Vector handframe_offset;
    handframe_offset[0]=0.1;handframe_offset[1]=0;handframe_offset[2]=0;
    KDL::Vector palmframe_offset= T_palm_hand*handframe_offset;
    KDL::Vector worldframe_offset=T_world_ee.M*palmframe_offset;
    T_world_ee.p += worldframe_offset;

   }  

    //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.

    // desired ee position wrt to robot body.
    //T_body_ee = T_body_world*T_world_ee;
    T_body_ee = T_world_ee; // send them in world frame for now.
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

    goalmsg.num_chain_joints  = sticky_hand_struc.joint_name.size();
    // No specified posture bias
    goalmsg.use_posture_bias  = false;
    goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
    goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
    for(int i = 0; i < goalmsg.num_chain_joints; i++){
      if(!pregrasp_flag){
        goalmsg.joint_posture_bias[i]=sticky_hand_struc.joint_position[i];
      }
      else{
        goalmsg.joint_posture_bias[i]=0;//sticky_hand_struc.joint_position[i];
      }
      goalmsg.chain_joint_names[i]= sticky_hand_struc.joint_name[i];
    }

    // Publish the message
    goalmsg.halt_ee_controller = false;

    _lcm->publish(channel, &goalmsg);
  }
  
  
  static void publish_desired_hand_motion( StickyHandStruc &sticky_hand_struc,std::string ee_name, std::string channel, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    drc::traj_opt_constraint_t trajmsg;
    trajmsg.utime = self->last_state_msg_timestamp;
    trajmsg.robot_name = "atlas";
    
    KDL::Frame  T_geometry_hand = sticky_hand_struc.T_geometry_hand;
    KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
   if(!sticky_hand_struc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
     cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
    KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset


    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(sticky_hand_struc.object_name));
    KDL::Frame T_world_object = obj_it->second._gl_object->_T_world_body;
    
      trajmsg.num_links =  sticky_hand_struc._gl_hand->_desired_body_motion_history.size();
        for(uint i = 0; i < (uint) trajmsg.num_links; i++)
        {
           double x,y,z,w;
           KDL::Frame T_object_hand = sticky_hand_struc._gl_hand->_desired_body_motion_history[i];
           KDL::Frame T_world_hand = T_world_object*T_object_hand;
           KDL::Frame nextTfframe = T_world_hand*T_hand_palm;//T_world_palm ; TODO: Eventually will be in object frame
           nextTfframe.M.GetQuaternion(x,y,z,w);

           drc::position_3d_t pose;
	         pose.translation.x = nextTfframe.p[0];
	         pose.translation.y = nextTfframe.p[1];
	         pose.translation.z = nextTfframe.p[2];
           pose.rotation.x = x;
           pose.rotation.y = y;
           pose.rotation.z = z;
           pose.rotation.w = w; 
           trajmsg.link_name.push_back(ee_name);
           trajmsg.link_origin_position.push_back(pose);  
           trajmsg.link_timestamps.push_back(i);     
	      }
	      
	      trajmsg.num_joints =0;
     self->lcm->publish(channel, &trajmsg);
  }
  
  
  static void publish_grasp_state_for_execution( StickyHandStruc &sticky_hand_struc,std::string ee_name, std::string channel,KDL::Frame &T_world_geometry, bool palmtouch_flag,bool getclose_flag, bool pregrasp_flag, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    drc::desired_grasp_state_t msg;
    msg.utime = self->last_state_msg_timestamp;
    msg.robot_name = "atlas";
    
    msg.object_name = string(sticky_hand_struc.object_name);
    msg.geometry_name = string(sticky_hand_struc.geometry_name);
    msg.unique_id = sticky_hand_struc.uid;
    msg.grasp_type = sticky_hand_struc.hand_type;

    // desired ee position in world frame
    KDL::Frame T_world_ee,T_body_ee;
    
// Must account for the mismatch between l_hand and base in sandia_hand urdf. Publish in palm frame.   
   KDL::Frame  T_geometry_hand = sticky_hand_struc.T_geometry_hand;
//    KDL::Frame T_hand_palm = KDL::Frame::Identity();
//    // this was there in urdf to make sure fingers are pointing in z axis.
//    T_hand_palm.M =  KDL::Rotation::RPY(0,-(M_PI/2),0); 
//    KDL::Frame  T_geometry_palm = T_geometry_hand*T_hand_palm.Inverse()

    KDL::Frame  T_geometry_palm = KDL::Frame::Identity(); 
    if(!sticky_hand_struc._gl_hand->get_link_frame(ee_name,T_geometry_palm))
      cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;

//    double ro,pi,ya;  
//   KDL::Frame T_hand_palm = T_geometry_hand.Inverse()*T_geometry_palm; // offset
//   T_hand_palm.M.GetRPY(ro,pi,ya);
//    cout <<"pitch"<<pi*(180/M_PI) << endl;
    
    T_world_ee = T_world_geometry*T_geometry_palm;
          
   
    KDL::Frame T_palm_hand = T_geometry_palm.Inverse()*T_geometry_hand; // offset
    
    
    KDL::Frame T_hand_offset = KDL::Frame::Identity();
    
    if(palmtouch_flag){
      T_hand_offset.p[0] += 0.001;// 2cm  away from object 
    }  
    if(pregrasp_flag){
      T_hand_offset.p[0] += 0.3;  // 20cm  move away from which ever direction the palm is facing by 10 cm 
    }  
    if(getclose_flag){
     T_hand_offset.p[0] += 0.09; 
    }
    // The palm frame is pointing in negative x axis. This is a convention for sticky hands.
    KDL::Frame T_palm_offset =  T_palm_hand*T_hand_offset;
    
    // cout <<"before offset"<<T_world_ee.p[0]<<" "<<T_world_ee.p[1]<<" "<<T_world_ee.p[2] << endl;
    T_world_ee = T_world_geometry*T_geometry_palm*T_palm_offset;
    //cout <<"after offset"<<T_world_ee.p[0]<<" "<<T_world_ee.p[1]<<" "<<T_world_ee.p[2] << endl;

 
// double ro,pi,ya;
//   T_world_ee.M.GetRPY(ro,pi,ya);
//   cout <<"roll"<<ro*(180/M_PI) << endl;
//   cout <<"pitch"<<pi*(180/M_PI) << endl;
//   cout <<"yaw"<<ya*(180/M_PI) << endl;
   
          
    //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.

    // desired ee position wrt to robot body.
    //T_body_ee = T_body_world*T_world_ee;
    T_body_ee = T_world_ee; // send them in world frame for now.
    double x,y,z,w;
    T_body_ee.M.GetQuaternion(x,y,z,w);
    
    drc::position_3d_t hand_pose; 

    hand_pose.translation.x = T_body_ee.p[0];
    hand_pose.translation.y = T_body_ee.p[1];
    hand_pose.translation.z = T_body_ee.p[2];

    hand_pose.rotation.x = x;
    hand_pose.rotation.y = y;
    hand_pose.rotation.z = z;
    hand_pose.rotation.w = w;
    
    if((msg.grasp_type == msg.SANDIA_LEFT)||(msg.grasp_type == msg.IROBOT_LEFT)){
      msg.l_hand_pose = hand_pose;
      msg.num_l_joints  = sticky_hand_struc.joint_name.size();
      msg.num_r_joints  = 0;
      msg.l_joint_name.resize(msg.num_l_joints);
      msg.l_joint_position.resize(msg.num_l_joints);
      for(int i = 0; i < msg.num_l_joints; i++){
        if((!palmtouch_flag)&&(!pregrasp_flag)&&(!getclose_flag)){
          msg.l_joint_position[i]=sticky_hand_struc.joint_position[i];
        }
        else{
          msg.l_joint_position[i]=0;//sticky_hand_struc.joint_position[i];
        }
        msg.l_joint_name[i]= sticky_hand_struc.joint_name[i];
      }
    }
    else if((msg.grasp_type == msg.SANDIA_RIGHT)||(msg.grasp_type == msg.IROBOT_RIGHT)){
      msg.r_hand_pose = hand_pose;
      msg.num_r_joints  = sticky_hand_struc.joint_name.size();
      msg.num_l_joints  = 0;
      msg.r_joint_name.resize(msg.num_r_joints);
      msg.r_joint_position.resize(msg.num_r_joints);
      for(int i = 0; i < msg.num_r_joints; i++){
        if((!palmtouch_flag)&&(!pregrasp_flag)&&(!getclose_flag)){
          msg.r_joint_position[i]=sticky_hand_struc.joint_position[i];
        }
        else{
          msg.r_joint_position[i]=0;//sticky_hand_struc.joint_position[i];
        }
        msg.r_joint_name[i]= sticky_hand_struc.joint_name[i];
      }
    }

    // Publish the message 
    self->lcm->publish(channel, &msg);
  }
  
  
  static void on_sticky_hand_dblclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    
    if(self->stickyhand_selection==" "){
      gtk_widget_destroy(self->dblclk_popup);
      return;
    }
    
    if (! strcmp(name, PARAM_DELETE)) {
      fprintf(stderr,"\n Clearing selected sticky hand\n");
      
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find(self->stickyhand_selection);
      if(hand_it!=self->sticky_hands.end())
        self->sticky_hands.erase(hand_it);
      self->stickyhand_selection = " ";
      bot_viewer_request_redraw(self->viewer);
    }
    else if (! strcmp(name, PARAM_RESEED)) {
      cout << "TODO" << endl;
    }
    else if(! strcmp(name, PARAM_EE_MOTION)) {
    
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find(self->stickyhand_selection);
      
      //===================================// TODO: //DEBUG ONLY.REMOVE BLOCK LATER
     typedef map<string, OtdfInstanceStruc > object_instance_map_type_;

      object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(hand_it->second.object_name));
      KDL::Frame T_world_graspgeometry_future = KDL::Frame::Identity(); // the object might have moved.

      if(!obj_it->second._gl_object->get_link_geometry_future_frame(string(hand_it->second.geometry_name),T_world_graspgeometry_future))
        cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
      else { 
      //===================================// TODO: //DEBUG ONLY.REMOVE BLOCK LATER
      
      drc::grasp_opt_control_t msg; // just to access types
      int grasp_type = hand_it->second.hand_type;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH; 
        //publish ee goal msg.
        if(grasp_type == msg.SANDIA_LEFT){
          publish_desired_hand_motion(hand_it->second,"left_palm","DESIRED_LEFT_PALM_MOTION",self);          
         // publish_grasp_state_for_execution(hand_it->second,"left_palm","COMMITTED_GRASP_SEED",T_world_graspgeometry_future,false,false,false,self);//DEBUG ONLY. TODO: REMOVE LATER
         }
        else if(grasp_type== msg.SANDIA_RIGHT){
          publish_desired_hand_motion( hand_it->second,"right_palm","DESIRED_RIGHT_PALM_MOTION",self);
          //DEBUG ONLY. TODO: REMOVE LATER
         // publish_grasp_state_for_execution(hand_it->second,"right_palm","COMMITTED_GRASP_SEED",T_world_graspgeometry_future,false,false,false,self);//DEBUG ONLY. TODO: REMOVE LATER
        }
        
      }// REMOVE LATER
      
    }
    else if (!strcmp(name, PARAM_COMMIT)) {
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find(self->stickyhand_selection);

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(hand_it->second.object_name));
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.

      if(!obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
        cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
      else { 
        drc::desired_grasp_state_t msg; // just to access types
        int grasp_type = hand_it->second.hand_type;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH; 

        //publish desired_grasp_state_t on COMMITED_GRASP msg.
            //publish ee goal msg.
        if(grasp_type == msg.SANDIA_LEFT)
          publish_grasp_state_for_execution(hand_it->second,"left_palm","COMMITTED_GRASP_SEED",T_world_graspgeometry,false,false,false,self);
        else if(grasp_type== msg.SANDIA_RIGHT)
          publish_grasp_state_for_execution(hand_it->second,"right_palm","COMMITTED_GRASP_SEED",T_world_graspgeometry,false,false,false,self);
      }
     
    }
    else if ((!strcmp(name, PARAM_EXECUTE))||(!strcmp(name, PARAM_PALM_TOUCH))||(!strcmp(name, PARAM_PREGRASP))||(!strcmp(name, PARAM_GET_CLOSE))) {
    
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find(self->stickyhand_selection);

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(string(hand_it->second.object_name));
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.

      if(!obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
      cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
      else { 
        drc::grasp_opt_control_t msg; // just to access types
        int grasp_type = hand_it->second.hand_type;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH; 
        
        bool getclose_flag = !strcmp(name, PARAM_GET_CLOSE);
        bool palmtouch_flag = !strcmp(name, PARAM_PALM_TOUCH);
        bool pregrasp_flag = !strcmp(name, PARAM_PREGRASP);

        //publish ee goal msg.
        if(grasp_type == msg.SANDIA_LEFT) {
           publish_eegoal_to_sticky_hand(self->lcm, hand_it->second,"left_palm","LEFT_PALM_GOAL",T_world_graspgeometry,pregrasp_flag);
           if(palmtouch_flag||getclose_flag||pregrasp_flag){ //DEBUG ONLY. TODO: REMOVE LATER
            publish_grasp_state_for_execution(hand_it->second,"left_palm","COMMITTED_GRASP_SEED",T_world_graspgeometry,palmtouch_flag,getclose_flag,pregrasp_flag,self);
           }
        }
        else if(grasp_type== msg.SANDIA_RIGHT) {
          publish_eegoal_to_sticky_hand(self->lcm, hand_it->second,"right_palm","RIGHT_PALM_GOAL",T_world_graspgeometry,pregrasp_flag);
           if(palmtouch_flag||getclose_flag||pregrasp_flag){ //DEBUG ONLY.  TODO: REMOVE LATER
            publish_grasp_state_for_execution(hand_it->second,"right_palm","COMMITTED_GRASP_SEED",T_world_graspgeometry,palmtouch_flag,getclose_flag,pregrasp_flag,self);
           }
        }
      }
 
    }
    else if (! strcmp(name, PARAM_HALT_OPT)) {
       cout << "TODO" << endl;
    }
        
    bot_viewer_request_redraw(self->viewer);
    gtk_widget_destroy(self->dblclk_popup);
    
   }
  
  static void spawn_sticky_hand_dblclk_popup (RendererAffordances *self)
  {
    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw;

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
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    bot_gtk_param_widget_add_buttons(pw,PARAM_DELETE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_RESEED, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_PREGRASP,NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_GET_CLOSE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_PALM_TOUCH, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_COMMIT, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_EXECUTE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_EE_MOTION, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_HALT_OPT, NULL);

    
    //cout <<self->selection << endl; // otdf_type::geom_name
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_sticky_hand_dblclk_popup_param_widget_changed), self);

    self->dblclk_popup  = window;

    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),"clicked",G_CALLBACK (on_popup_close),(gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy", G_CALLBACK(on_dblclk_popup_close), self); 


    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
      gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 

  }
  

  
  

}
#endif
