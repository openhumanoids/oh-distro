#ifndef STICKYHAND_INTERACTION_GUI_UTILS_HPP
#define STICKYHAND_INTERACTION_GUI_UTILS_HPP

#include "object_interaction_gui_utils.hpp"

using namespace renderer_affordances;
using namespace renderer_affordances_lcm_utils;

namespace renderer_affordances_gui_utils
{

  //--------------------------------------------------------------------------
  // Sticky Hand Interaction
  //

  static void on_sticky_hand_dblclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    
    if(self->stickyhand_selection==" "){
      gtk_widget_destroy(self->dblclk_popup);
      return;
    }
    
    if (! strcmp(name, PARAM_DELETE)) {
      fprintf(stderr,"\n Clearing selected sticky hand\n");
      self->stickyHandCollection->remove(self->stickyhand_selection);
      self->stickyhand_selection = " ";
      bot_viewer_request_redraw(self->viewer);
    }
    else if (! strcmp(name, PARAM_RESEED)) {
      cout << "TODO" << endl;
    }
    else if(! strcmp(name, PARAM_MOVE_EE)) {
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);

      drc::grasp_opt_control_t msg; // just to access types
      int grasp_type = hand_it->second.hand_type;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH; 
        //publish ee goal msg.
        if(grasp_type == msg.SANDIA_LEFT){
          publish_desired_hand_motion(hand_it->second,"left_palm","DESIRED_LEFT_PALM_MOTION",self);
         }
        else if(grasp_type== msg.SANDIA_RIGHT){
          publish_desired_hand_motion( hand_it->second,"right_palm","DESIRED_RIGHT_PALM_MOTION",self);
        }
    }
    
    else if ((!strcmp(name, PARAM_GRASP_UNGRASP))||(!strcmp(name, PARAM_POWER_GRASP))) {
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);
      
      bool val = (hand_it->second.grasp_status==0); // is just a candidate, enable grasp

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(hand_it->second.object_name));
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.

      if(!obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
        cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
      else { 
        drc::desired_grasp_state_t msg; // just to access types
        int grasp_type = hand_it->second.hand_type;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH; 
        
        bool power_flag = !strcmp(name, PARAM_POWER_GRASP);
        if(power_flag)
         val=true;
  

        //publish desired_grasp_state_t on COMMITED_GRASP msg.
            //publish ee goal msg.
        if(grasp_type == msg.SANDIA_LEFT)
          publish_grasp_state_for_execution(hand_it->second,"left_palm","COMMITTED_GRASP",T_world_graspgeometry,val,power_flag,self);
        else if(grasp_type== msg.SANDIA_RIGHT)
          publish_grasp_state_for_execution(hand_it->second,"right_palm","COMMITTED_GRASP",T_world_graspgeometry,val,power_flag,self);
          
        hand_it->second.grasp_status = !hand_it->second.grasp_status;  
      }
     
    }
    else if (!strcmp(name, PARAM_PARTIAL_GRASP_UNGRASP)) {
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);
      int g_status = bot_gtk_param_widget_get_enum(pw, PARAM_PARTIAL_GRASP_UNGRASP);
      fprintf(stderr, "Requested grasp status : %d\n", g_status);
      //set the value 
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(hand_it->second.object_name));
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.

      if(!obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
        cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
      else { 
        drc::desired_grasp_state_t msg; // just to access types
        //publish desired_grasp_state_t on COMMITED_GRASP msg.
            //publish ee goal msg.
        int grasp_type = hand_it->second.hand_type;
        if(grasp_type == msg.SANDIA_LEFT)
          publish_partial_grasp_state_for_execution(hand_it->second,"left_palm","COMMITTED_GRASP",T_world_graspgeometry, g_status, self);
        else if(grasp_type== msg.SANDIA_RIGHT)
          publish_partial_grasp_state_for_execution(hand_it->second,"right_palm","COMMITTED_GRASP",T_world_graspgeometry, g_status, self);
          
        //hand_it->second.grasp_status = !hand_it->second.grasp_status;  
        hand_it->second.partial_grasp_status = g_status; 
      }
    }
    else if(!strcmp(name,PARAM_SEND_POSE_GOAL3)){
      string channel = "POSE_GOAL";
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);
      KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
      publish_pose_goal_to_sticky_hand(self,channel,hand_it->second,T_world_body_desired,false);  
    }

    else if(!strcmp(name,PARAM_SEND_POSE_GOAL4)){
      string channel = "POSE_GOAL";
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);
      KDL::Frame T_world_body_desired = self->robotStateListener->T_body_world.Inverse();
      publish_pose_goal_to_sticky_hand(self,channel,hand_it->second,T_world_body_desired,true);  
    }

    else if ((!strcmp(name, PARAM_TOUCH))||(!strcmp(name, PARAM_REACH))) {
    
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(hand_it->second.object_name));
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.

      if(!obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_graspgeometry))
      cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
      else { 
        drc::grasp_opt_control_t msg; // just to access types
        int grasp_type = hand_it->second.hand_type;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH; 

        bool reach_flag = !strcmp(name, PARAM_REACH);

        //publish ee goal msg.
        if(grasp_type == msg.SANDIA_LEFT) {
           publish_eegoal_to_sticky_hand(self->lcm, hand_it->second,"left_palm","LEFT_PALM_GOAL",T_world_graspgeometry,reach_flag);
        }
        else if(grasp_type== msg.SANDIA_RIGHT) {
          publish_eegoal_to_sticky_hand(self->lcm, hand_it->second,"right_palm","RIGHT_PALM_GOAL",T_world_graspgeometry,reach_flag);

        }
      }
 
    }
    else if (! strcmp(name, PARAM_HALT_OPT)) {
       cout << "TODO" << endl;
    }
    else if ((!strcmp(name, PARAM_STORE))) {
      self->stickyHandCollection->store(self->stickyhand_selection,false,self->affCollection);
      //store_sticky_hand(pw,name,user,false);    
    }
    else if ((!strcmp(name, PARAM_UNSTORE))) {
      self->stickyHandCollection->store(self->stickyhand_selection,true,self->affCollection);
      //store_sticky_hand(pw,name,user,true);    
    }
    else if (!strcmp(name, PARAM_MELD_HAND_TO_CURRENT))
    {
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);
      if(!hand_it->second.is_melded)
      {
        //change joint_position to current hand state and 
        //T_geometry_hand to T_geometry_world*T_world_palm(from FK)*T_palm_stickyhandbase;   
      
        hand_it->second.optimized_joint_position = hand_it->second.joint_position;  
        hand_it->second.optimized_T_geometry_hand = hand_it->second.T_geometry_hand;
        
        if(self->robotStateListener->_urdf_parsed) 
        {
          for (size_t k=0;k<hand_it->second.joint_name.size();k++)
          {
            double pos;
            bool val= self->robotStateListener->_gl_robot->get_current_joint_pos(hand_it->second.joint_name[k],pos);
            hand_it->second.joint_position[k] = pos;
            cout << hand_it->second.joint_name[k] << " pos:" << pos << " flag:"<< val << endl;
          }// end for
          drc::joint_angles_t posture_msg;
          posture_msg.num_joints= hand_it->second.joint_name.size();
          posture_msg.joint_name = hand_it->second.joint_name;
          posture_msg.joint_position = hand_it->second.joint_position; 
          
          typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
          object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(hand_it->second.object_name));
          KDL::Frame T_world_geometry = KDL::Frame::Identity(); // the object might have moved.
          if(!obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_geometry))
            cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;           
          
          KDL::Frame T_world_palm, T_geometry_stickyhandbase,T_geometry_palm,T_palm_stickyhandbase; 
          std::string ee_name;
          if(hand_it->second.hand_type==0)
             ee_name = "left_palm";
          else
             ee_name = "right_palm";
          T_geometry_stickyhandbase = hand_it->second._gl_hand->_T_world_body;
          hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_palm);         
          T_palm_stickyhandbase = T_geometry_palm.Inverse()*T_geometry_stickyhandbase;
          
          self->robotStateListener->_gl_robot->get_link_frame(ee_name,T_world_palm);
          KDL::Frame T_geometry_palm_new = T_world_geometry.Inverse()*T_world_palm;
          KDL::Frame T_geometry_stickyhandbase_new = T_geometry_palm_new*T_palm_stickyhandbase;
          cout << "setting sticky hand state to current hand pose and posture " << endl;
          hand_it->second._gl_hand->set_state(T_geometry_stickyhandbase_new, posture_msg);
          hand_it->second.T_geometry_hand = T_geometry_stickyhandbase_new;
          
        } // end if
           // 
      } 
      else {
        cout << "resetting  sticky hand state to optimized hand pose and posture " << endl;
        hand_it->second.T_geometry_hand = hand_it->second.optimized_T_geometry_hand;
        hand_it->second.joint_position = hand_it->second.optimized_joint_position; // reset;
        drc::joint_angles_t posture_msg;
        posture_msg.num_joints= hand_it->second.joint_name.size();
        posture_msg.joint_name = hand_it->second.joint_name;
        posture_msg.joint_position = hand_it->second.joint_position;   
        KDL::Frame T_geometry_stickyhandbase = hand_it->second.T_geometry_hand;
        hand_it->second._gl_hand->set_state(T_geometry_stickyhandbase, posture_msg); 
      } 
  	  // toggle  flag
      hand_it->second.is_melded = !hand_it->second.is_melded; 
    } // end if else
    else if ((!strcmp(name,  PARAM_MELD_PARENT_AFF_TO_ESTROBOTSTATE))) {
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(hand_it->second.object_name));
      if(hand_it->second.is_melded)
      {
        obj_it->second.is_melded = !obj_it->second.is_melded; 
      }
    }
  
    bot_viewer_request_redraw(self->viewer);
    gtk_widget_destroy(self->dblclk_popup);
    
   }
  //--------------------------------------------------------------------------
  static void spawn_sticky_hand_dblclk_popup (RendererAffordances *self)
  {
  
    typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
    sticky_hands_map_type_::iterator hand_it = self->stickyHandCollection->_hands.find(self->stickyhand_selection);
  
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
    bot_gtk_param_widget_add_buttons(pw,PARAM_REACH,NULL);
    //bot_gtk_param_widget_add_buttons(pw,PARAM_GET_CLOSE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_TOUCH, NULL);
    //bot_gtk_param_widget_add_buttons(pw,PARAM_GRASP_UNGRASP, NULL);
    bool val = (hand_it->second.grasp_status==0);
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_GRASP_UNGRASP, !val, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_POWER_GRASP, NULL);
    int p_val = (hand_it->second.partial_grasp_status);
    

    bot_gtk_param_widget_add_buttons(pw,PARAM_MOVE_EE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_STORE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_UNSTORE, NULL);
    //bot_gtk_param_widget_add_buttons(pw,PARAM_HALT_OPT, NULL);

    bot_gtk_param_widget_add_separator(pw, "Partial Grasp");
    bot_gtk_param_widget_add_enum(pw, PARAM_PARTIAL_GRASP_UNGRASP, BOT_GTK_PARAM_WIDGET_MENU, p_val, "Ungrasped", 0, "Partial Grasp", 1, "Full Grasp", 2, "Grasp w/o Thumb", 3, NULL);
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->affCollection->_objects.find(string(hand_it->second.object_name));
  
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEND_POSE_GOAL3, NULL);
    
    //if(obj_it->second._gl_object->is_future_display_active())
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEND_POSE_GOAL4, NULL);

    val  = hand_it->second.is_melded;
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_MELD_HAND_TO_CURRENT, val, NULL);
    

    val  = ((hand_it->second.is_melded)&&(obj_it->second.is_melded));
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON,  PARAM_MELD_PARENT_AFF_TO_ESTROBOTSTATE, val, NULL);
    
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
 //=======================================================================================  
  

}
#endif
