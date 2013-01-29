#ifndef OBJECT_INTERACTION_GUI_UTILS_HPP
#define OBJECT_INTERACTION_GUI_UTILS_HPP

#define PARAM_SEED_LH "Seed LHand"
#define PARAM_SEED_RH "Seed RHand"
#define PARAM_SEED_LF "Seed LFoot"
#define PARAM_SEED_RF "Seed RFoot"
#define PARAM_HALT_ALL_OPT "Halt All Opts"

#define PARAM_COMMIT "Publish eegoal"
#define PARAM_RESEED "Re-seed"
#define PARAM_HALT_OPT "Halt Opt"
#define PARAM_DELETE "Delete"
#define PARAM_ENABLE_ADJUSTMENT "Adjust Pose"
#define PARAM_CONTACT_MASK_SELECT "Mask"

using namespace renderer_affordances;

namespace renderer_affordances_gui_utils
{

//  OTDF object rightclk popup and associated cbs.
  static void on_object_geometry_dblclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
  
    if (! strcmp(name, PARAM_ENABLE_ADJUSTMENT)) {
      bool val = bot_gtk_param_widget_get_bool(pw, PARAM_ENABLE_ADJUSTMENT);
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it= self->instantiated_objects.find((*self->object_selection));
      std::cout << "enabling link adjustment for object " <<(*self->object_selection) << " to "<< val << std::endl;
      if(it!=self->instantiated_objects.end()){
        it->second._gl_object->enable_link_adjustment(val);  
      }
    }
    else if (! strcmp(name, PARAM_SEED_LH)) {
      KDL::Frame T_geom_lhandpose = self->T_graspgeometry_handinitpos;
      KDL::Frame T_geom_rhandpose = KDL::Frame::Identity(); 
      drc::grasp_opt_control_t msg;

      int grasp_type = msg.SANDIA_LEFT;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;
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
      KDL::Frame T_geom_lhandpose = KDL::Frame::Identity();
      KDL::Frame T_geom_rhandpose = self->T_graspgeometry_handinitpos;
      
      //T_geom_rhandpose = KDL::Frame::Identity();
      //T_geom_rhandpose.M =  KDL::Rotation::RPY((M_PI/4),0,(M_PI/4));
      //double x,y,z,w;
      //T_geom_rhandpose.M.GetQuaternion(x,y,z,w);
      //std::cout << w << " " << x <<" " << y << " " << z << std::endl;
      drc::grasp_opt_control_t msg;
   
      int grasp_type = msg.SANDIA_RIGHT;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH;
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
        
    bot_viewer_request_redraw(self->viewer);
    if(strcmp(name, PARAM_CONTACT_MASK_SELECT))
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
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "dblclk");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    
    drc::grasp_opt_control_t msg;
    bot_gtk_param_widget_add_separator (pw,"Contact Filter");
    int num_masks =  2;
    char ** contact_masks =(char **) calloc(num_masks, sizeof(char *));
    int* contact_nums = (int *)calloc(num_masks, sizeof(int));
    contact_masks[0]=(char*) "ALL"; contact_masks[1]=(char*) "FINGERS";
    contact_nums[0]=msg.ALL; contact_nums[1]=msg.FINGERS_ONLY;
    bot_gtk_param_widget_add_enumv (pw, PARAM_CONTACT_MASK_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                          msg.ALL,
				                          num_masks,
			                            (const char **)  contact_masks,
			                            contact_nums);
			                            
    bot_gtk_param_widget_add_separator (pw,"Opt Control");
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_LH, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_RH, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_LF, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_RF, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_HALT_ALL_OPT, NULL);
    
    
    
     typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
     object_instance_map_type_::iterator it= self->instantiated_objects.find((*self->object_selection));
     bool val;
     if(it!=self->instantiated_objects.end())
      val = it->second._gl_object->is_link_adjustment_enabled();
     else
      val =false;
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_ENABLE_ADJUSTMENT, val, NULL);
    
    
    //cout <<*self->selection << endl; // otdf_type::geom_name
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
//  static void publish_eegoal_to_sticky_hand(boost::shared_ptr<lcm::LCM> &_lcm, StickyHandStruc &sticky_hand_struc,std::string ee_name, std::string channel,KDL::Frame &T_world_geometry)
//  {
//    drc::ee_goal_t goalmsg;
//    goalmsg.robot_name = "atlas";
//    goalmsg.root_name = "pelvis";
//    goalmsg.ee_name = ee_name;

//    double x,y,z,w;

//    // desired ee position in world frame
//    KDL::Frame T_world_ee,T_body_ee,T_geometry_hand;
//    T_geometry_hand = sticky_hand_struc.T_geometry_hand;
//    T_world_ee = T_world_geometry*T_geometry_hand;
//          
//    //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.

//    // desired ee position wrt to robot body.
//    //T_body_ee = T_body_world*T_world_ee;
//    T_body_ee = T_world_ee; // send them in world frame for now.

//    T_body_ee.M.GetQuaternion(x,y,z,w);

//    goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
//    goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
//    goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

//    goalmsg.ee_goal_pos.rotation.x = x;
//    goalmsg.ee_goal_pos.rotation.y = y;
//    goalmsg.ee_goal_pos.rotation.z = z;
//    goalmsg.ee_goal_pos.rotation.w = w;

//    goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
//    goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
//    goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
//    goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
//    goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
//    goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

//    goalmsg.num_chain_joints  = sticky_hand_struc.joint_name.size();
//    // No specified posture bias
//    goalmsg.use_posture_bias  = false;
//    goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
//    goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
//    for(int i = 0; i < goalmsg.num_chain_joints; i++){
//    goalmsg.joint_posture_bias[i]=sticky_hand_struc.joint_position[i];
//    goalmsg.chain_joint_names[i]= sticky_hand_struc.joint_name[i];
//    }

//    // Publish the message
//    goalmsg.halt_ee_controller = false;

//    _lcm->publish(channel, &goalmsg);
//  }
  
  static void on_sticky_hand_dblclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    
    if (! strcmp(name, PARAM_DELETE)) {
      fprintf(stderr,"\n Clearing selected sticky hand\n");
      
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find((*self->stickyhand_selection));
      if(hand_it!=self->sticky_hands.end())
        self->sticky_hands.erase(hand_it);
      (*self->stickyhand_selection) = " ";
      bot_viewer_request_redraw(self->viewer);
    }
    else if (! strcmp(name, PARAM_RESEED)) {
      cout << "TODO" << endl;
    }
    else if (! strcmp(name, PARAM_COMMIT)) {
    
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.find((*self->stickyhand_selection));

      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(hand_it->second.object_name);
      KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.

      if(!obj_it->second._gl_object->get_link_frame(hand_it->second.geometry_name,T_world_graspgeometry))
      cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
      else { 
        drc::grasp_opt_control_t msg; // just to access types
        int grasp_type = hand_it->second.hand_type;//or SANDIA_RIGHT,SANDIA_BOTH,IROBOT_LEFT,IROBOT_RIGHT,IROBOT_BOTH; 
        
        //publish ee goal msg.
//        if(grasp_type == msg.SANDIA_LEFT)
//          publish_eegoal_to_sticky_hand(self->lcm, hand_it->second,"l_hand","L_HAND_GOAL",T_world_graspgeometry);
//        else if(grasp_type== msg.SANDIA_RIGHT)
//          publish_eegoal_to_sticky_hand(self->lcm, hand_it->second,"r_hand","R_HAND_GOAL",T_world_graspgeometry);
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
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "dblclk");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    bot_gtk_param_widget_add_buttons(pw,PARAM_DELETE, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_RESEED, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_COMMIT, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_HALT_OPT, NULL);
    
    
    //cout <<*self->selection << endl; // otdf_type::geom_name
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
