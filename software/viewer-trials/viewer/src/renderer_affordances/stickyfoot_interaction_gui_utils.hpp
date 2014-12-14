#ifndef STICKYFOOT_INTERACTION_GUI_UTILS_HPP
#define STICKYFOOT_INTERACTION_GUI_UTILS_HPP

#define PARAM_FOOT_STATIC_TOGGLE "Static"



#include "object_interaction_gui_utils.hpp"

using namespace renderer_affordances;
using namespace renderer_affordances_lcm_utils;

namespace renderer_affordances_gui_utils
{

   static void spawn_sticky_foot_set_condition_popup (RendererAffordances *self);
  //--------------------------------------------------------------------------
  // Sticky Foot Interaction
  //
  static void on_sticky_foot_dblclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
      RendererAffordances *self = (RendererAffordances*) user;
      
      if(self->stickyfoot_selection==" "){
          gtk_widget_destroy(self->dblclk_popup);
          return;
      }
      
      if (! strcmp(name, PARAM_DELETE)) {
          fprintf(stderr,"\n Clearing selected sticky foot\n");
          self->stickyFootCollection->remove(self->stickyfoot_selection);
          self->stickyfoot_selection = " ";
          bot_viewer_request_redraw(self->viewer);
      }
      else if(! strcmp(name, PARAM_MOVE_EE)) {
          
          typedef map<string, StickyFootStruc > sticky_feet_map_type_;
          sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
          
          drc::grasp_opt_control_t msg; // just to access types
          int grasp_type = foot_it->second.foot_type;// 0 for left and 1 for right
          //publish ee goal msg.
          if(grasp_type == 0)
              publish_desired_foot_motion(foot_it->second,"l_foot","DESIRED_L_FOOT_MOTION",self);
          else if(grasp_type== 1)
              publish_desired_foot_motion( foot_it->second,"r_foot","DESIRED_R_FOOT_MOTION",self);
      }
      else if ((!strcmp(name, PARAM_TOUCH))||(!strcmp(name, PARAM_REACH))) {
          
          typedef map<string, StickyFootStruc > sticky_feet_map_type_;
          sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
          
          typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
          object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(foot_it->second.object_name));
          KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.
          
          if(!obj_it->second._gl_object->get_link_geometry_frame(string(foot_it->second.geometry_name),T_world_graspgeometry))
              cerr << " failed to retrieve " << foot_it->second.geometry_name<<" in object " << foot_it->second.object_name <<endl;
          else { 
              // just to access types
              int foot_type = foot_it->second.foot_type;// 0 for left and 1 for right
              bool touch_flag = !strcmp(name, PARAM_TOUCH);
              bool reach_flag = !strcmp(name, PARAM_REACH);
              
              //publish ee goal msg.
              if(foot_type == 0)
                  publish_eegoal_to_sticky_foot(self->lcm, foot_it->second,"l_foot","L_FOOT_GOAL",T_world_graspgeometry,reach_flag);
              else if(foot_type== 1)
                  publish_eegoal_to_sticky_foot(self->lcm, foot_it->second,"r_foot","R_FOOT_GOAL",T_world_graspgeometry,reach_flag);
          }
      }
      else if(!strcmp(name,PARAM_SEND_POSE_GOAL4)){
        string channel = "POSE_GOAL";
        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
        KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
        publish_pose_goal_to_sticky_foot(self,channel,foot_it->second,T_world_body_desired,false,false);  
      }

      else if(!strcmp(name,PARAM_SEND_POSE_GOAL5)){
        string channel = "POSE_GOAL";
        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
        KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
        bool end_state_only = true;
        publish_pose_goal_to_sticky_foot(self,channel,foot_it->second,T_world_body_desired,true,end_state_only);  
      }
      
      else if(!strcmp(name,PARAM_SEND_POSE_GOAL6)){
        string channel = "POSE_GOAL";
        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
        KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
        bool end_state_only = false;
        publish_pose_goal_to_sticky_foot(self,channel,foot_it->second,T_world_body_desired,true,end_state_only);  
      } 
      else if (!strcmp(name, PARAM_MELD_FOOT_TO_CURRENT))
      {
        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
        if(!foot_it->second.is_melded)
        {
          //change joint_position to current foot state and 
          //T_geometry_foot to T_geometry_world*T_world_foot(from FK);   
        
          foot_it->second.optimized_joint_position = foot_it->second.joint_position;  
          foot_it->second.optimized_T_geometry_foot = foot_it->second.T_geometry_foot;
          
          if(self->robotStateListener->_urdf_parsed) 
          {
            for (size_t k=0;k<foot_it->second.joint_name.size();k++)
            {
              double pos;
              bool val= self->robotStateListener->_gl_robot->get_current_joint_pos(foot_it->second.joint_name[k],pos);
              foot_it->second.joint_position[k] = pos;
              cout << foot_it->second.joint_name[k] << " pos:" << pos << " flag:"<< val << endl;
            }// end for
            drc::joint_angles_t posture_msg;
            posture_msg.num_joints= foot_it->second.joint_name.size();
            posture_msg.joint_name = foot_it->second.joint_name;
            posture_msg.joint_position = foot_it->second.joint_position; 
            
            typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
            object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(foot_it->second.object_name));
            KDL::Frame T_world_geometry = KDL::Frame::Identity(); // the object might have moved.
            if(!obj_it->second._gl_object->get_link_geometry_frame(string(foot_it->second.geometry_name),T_world_geometry))
              cerr << " failed to retrieve " << foot_it->second.geometry_name<<" in object " << foot_it->second.object_name <<endl;           
              KDL::Frame T_world_ankle, T_geometry_stickyfootbase,T_foot_stickyfootbase;

            std::string joint_name;
            if(foot_it->second.foot_type==0){
               joint_name = "l_leg_aky";
            }
            else {
               joint_name = "r_leg_aky";
             }
            // stickyfoot base is a dummy link at  ankle joint origin
            T_geometry_stickyfootbase = foot_it->second._gl_foot->_T_world_body;

            JointFrameStruct jointinfo_struct;
            self->robotStateListener->_gl_robot->get_joint_info(joint_name,jointinfo_struct);
            T_world_ankle =jointinfo_struct.frame;
            KDL::Frame T_geometry_stickyfootbase_new = T_world_geometry.Inverse()*T_world_ankle;
            cout << "setting sticky foot state to current foot pose and posture " << endl;
            foot_it->second._gl_foot->set_state(T_geometry_stickyfootbase_new, posture_msg);
			foot_it->second.T_geometry_foot = T_geometry_stickyfootbase_new;
          } // end if
        } 
        else {
          cout << "resetting  sticky foot state to optimized foot pose and posture " << endl;
          foot_it->second.T_geometry_foot = foot_it->second.optimized_T_geometry_foot;
          foot_it->second.joint_position = foot_it->second.optimized_joint_position; // reset;
       
          drc::joint_angles_t posture_msg;
          posture_msg.num_joints= foot_it->second.joint_name.size();
          posture_msg.joint_name = foot_it->second.joint_name;
          posture_msg.joint_position = foot_it->second.joint_position;   
          KDL::Frame T_geometry_stickyfootbase = foot_it->second.T_geometry_foot;
          foot_it->second._gl_foot->set_state(T_geometry_stickyfootbase, posture_msg); 
        } 
    	// toggle  flag
      	foot_it->second.is_melded = !foot_it->second.is_melded; 
     	 //foot_it->second._gl_foot->disable_future_display();   
      } // end if else
      
    else if ((!strcmp(name, PARAM_STORE))) {
      self->stickyFootCollection->store(self->stickyfoot_selection,false,self->affCollection); 
    }
    else if ((!strcmp(name, PARAM_UNSTORE))) {
      self->stickyFootCollection->store(self->stickyfoot_selection,true,self->affCollection);  
    }
    else if (! strcmp(name, PARAM_FOOT_STATIC_TOGGLE)) {
     typedef map<string, StickyFootStruc > sticky_feet_map_type_;
     sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
     foot_it->second.is_static = !foot_it->second.is_static;
        
    }
    else if (! strcmp(name, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT)) {
        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT);
      if(val){
        std::cout << "enabling bodypose adjustment for sticky hand " << self->stickyfoot_selection << std::endl;
      }
   
      if(foot_it!=self->stickyFootCollection->_feet.end()){
        foot_it->second._gl_foot->enable_bodypose_adjustment(val);
        foot_it->second._gl_foot->enable_bodyorparent_frame_rendering_of_floatingbase_markers(val);
        foot_it->second._gl_foot->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::THREE_D);
        foot_it->second._gl_foot->enable_jointdof_adjustment(false);    
      }
    }
    else if ((! strcmp(name, PARAM_UPDATE_SEED_CONDITION))||(! strcmp(name, PARAM_MAKE_SEED_CONDITIONAL))) {
       spawn_sticky_foot_set_condition_popup(self);
    }
    else if ((!strcmp(name, PARAM_REMOVE_SEED_COND))){
        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
        foot_it->second.is_conditional = false;
        foot_it->second.conditioned_parent_joint_name = " ";
        foot_it->second.conditioned_parent_joint_val = 0;
    }
        
      if(strcmp(name, PARAM_UPDATE_SEED_CONDITION)
       && strcmp(name, PARAM_MAKE_SEED_CONDITIONAL)      
       )
      bot_viewer_request_redraw(self->viewer);
      gtk_widget_destroy(self->dblclk_popup);
    
  }
   //-------------------------------------------------------------------------
  static void on_sticky_foot_set_condition_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    
    if(self->stickyfoot_selection==" "){
      return;
    }
    typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
    sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(foot_it->second.object_name));
    
    if ((!strcmp(name, PARAM_SEED_COND)))
    {
      int cond_type = bot_gtk_param_widget_get_enum(pw, PARAM_SEED_COND);
      cout << "selected cond type:" << cond_type << endl;
    }
    else if ((!strcmp(name, PARAM_SEED_COND_JOINT_NAME)))
    {
      int joint_ind = bot_gtk_param_widget_get_enum(pw, PARAM_SEED_COND_JOINT_NAME);
      std::vector<std::string > jnt_names;
      jnt_names = obj_it->second._gl_object->get_joint_names();
      cout << "selected joint name:" << jnt_names[(size_t)joint_ind]  << endl;   
    }
    else if ((!strcmp(name, PARAM_SEED_COND_JOINT_VAL)))
    {
      double joint_val = bot_gtk_param_widget_get_double(pw, PARAM_SEED_COND_JOINT_VAL);
      cout << "setting seed condition val:" << joint_val << endl;
    }
    else if ((!strcmp(name, PARAM_SET_SEED_COND)))
    {
    
     int joint_ind = bot_gtk_param_widget_get_enum(pw, PARAM_SEED_COND_JOINT_NAME);
      std::vector<std::string > jnt_names;
      jnt_names = obj_it->second._gl_object->get_joint_names();
      double joint_val = bot_gtk_param_widget_get_double(pw, PARAM_SEED_COND_JOINT_VAL);
      foot_it->second.is_conditional = true;
      int cond_type = bot_gtk_param_widget_get_enum(pw, PARAM_SEED_COND);
      foot_it->second.cond_type = cond_type;
      foot_it->second.conditioned_parent_joint_name = jnt_names[(size_t)joint_ind];
      foot_it->second.conditioned_parent_joint_val = joint_val*(M_PI/180);// TODO: handle prismatic and revolute differently
    }
    else if (!strcmp(name, PARAM_REMOVE_SEED_COND))
    {
      foot_it->second.is_conditional = false;
      foot_it->second.conditioned_parent_joint_name = " ";
      foot_it->second.conditioned_parent_joint_val = 0;
    }
    
   
    if(   strcmp(name,PARAM_SEED_COND)
       && strcmp(name, PARAM_SEED_COND_JOINT_NAME)
       && strcmp(name, PARAM_SEED_COND_JOINT_VAL)
       )  
    {
        gtk_widget_destroy(self->second_stage_popup);
        gtk_widget_destroy(self->dblclk_popup);  
    }
  }    
  //--------------------------------------------------------------------------
  
  static void spawn_sticky_foot_set_condition_popup (RendererAffordances *self)
  {

    typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
    sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
  
    GtkWidget *window, *close_button, *vbox, *hbox;
    

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
    gtk_window_set_title(GTK_WINDOW(window), "setcondition");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    
    
    BotGtkParamWidget *pw;
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    bot_gtk_param_widget_add_enum(pw, PARAM_SEED_COND, BOT_GTK_PARAM_WIDGET_MENU, 0, 
                              "GT", foot_it->second.GT, 
                              "LT", foot_it->second.LT, 
                               NULL);
    // get list of joint names from parent object
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(foot_it->second.object_name));
    std::vector<std::string > jnt_names;
    jnt_names = obj_it->second._gl_object->get_joint_names();

     
    vector<const char*> cjnt_names;
    vector<int> cjnt_nums;
    for(int i = 0; i < jnt_names.size(); ++i)
    {
       cjnt_names.push_back(jnt_names[i].c_str());
       cjnt_nums.push_back(i);
    }
    bot_gtk_param_widget_add_enumv (pw,PARAM_SEED_COND_JOINT_NAME, BOT_GTK_PARAM_WIDGET_MENU, 
                                  0,
                                  jnt_names.size(),
                                  &cjnt_names[0],
                                  &cjnt_nums[0]);  
    bot_gtk_param_widget_add_double(pw, PARAM_SEED_COND_JOINT_VAL, BOT_GTK_PARAM_WIDGET_SPINBOX,
                                      -360, 360, .1, 0);
                                      
    bot_gtk_param_widget_add_buttons(pw,PARAM_SET_SEED_COND, NULL);  
    bot_gtk_param_widget_add_buttons(pw,PARAM_REMOVE_SEED_COND, NULL);                          
                                      
 
    //cout <<self->selection << endl; // otdf_type::geom_name
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_sticky_foot_set_condition_popup_param_widget_changed), self);

    self->second_stage_popup  = window;
     
    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),"clicked",G_CALLBACK (on_popup_close),(gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy", G_CALLBACK(on_second_stage_popup_close), self); 

    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 

  }
  //--------------------------------------------------------------------------
  static void spawn_sticky_foot_dblclk_popup (RendererAffordances *self)
  {
  
    typedef std::map<std::string, StickyFootStruc > sticky_feet_map_type_;
    sticky_feet_map_type_::iterator foot_it = self->stickyFootCollection->_feet.find(self->stickyfoot_selection);
  
     if(self->stickyfoot_selection==" "){
      gtk_widget_destroy(self->dblclk_popup);
      return;
    } 
    
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
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
    gtk_window_set_title(GTK_WINDOW(window), "dblclk");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    bot_gtk_param_widget_add_buttons(pw,PARAM_DELETE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_REACH,NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_TOUCH, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_MOVE_EE, NULL); 
    
    
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEND_POSE_GOAL4, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEND_POSE_GOAL5, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEND_POSE_GOAL6, NULL);
       
    
    bot_gtk_param_widget_add_buttons(pw,PARAM_STORE, NULL); 
    bot_gtk_param_widget_add_buttons(pw,PARAM_UNSTORE, NULL); 
    

    bool val  = foot_it->second.is_melded;
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_MELD_FOOT_TO_CURRENT, val, NULL);
    
    val =  foot_it->second._gl_foot->is_bodypose_adjustment_enabled();
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_ENABLE_CURRENT_BODYPOSE_ADJUSTMENT, val, NULL);
    
    
    val = foot_it->second.is_static;
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_FOOT_STATIC_TOGGLE, val, NULL);
    
        
    /*if(foot_it->second.is_conditional)
    {
      bot_gtk_param_widget_add_buttons(pw,PARAM_UPDATE_SEED_CONDITION, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_REMOVE_SEED_COND, NULL);
    }
    else 
      bot_gtk_param_widget_add_buttons(pw,PARAM_MAKE_SEED_CONDITIONAL , NULL);*/
    
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_sticky_foot_dblclk_popup_param_widget_changed), self);
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
 //=======================================================================================
  

}
#endif
