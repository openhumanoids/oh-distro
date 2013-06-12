#ifndef PLAN_EXECUTION_GUI_UTILS_HPP
#define PLAN_EXECUTION_GUI_UTILS_HPP
#define PARAM_LEFT_ARM_CT_SELECT "Left arm  "
#define PARAM_RIGHT_ARM_CT_SELECT "Right arm  "
#define PARAM_LEFT_LEG_CT_SELECT "Left legs  "
#define PARAM_RIGHT_LEG_CT_SELECT "Right legs  "
#define PARAM_STOP_WALKING "Stop walking"


using namespace renderer_robot_plan;  
namespace renderer_robot_plan_gui_utils
{
 //==================================================================================================  
  static gboolean on_popup_close (GtkButton* button, GtkWidget* pWindow)
  {
      gtk_widget_destroy (pWindow);
      return TRUE;
  }

  static void on_control_type_selection_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout <<"A compliant robot plan approved" << endl;
    cout <<"Publishing a compliant robot plan on COMMITTED_ROBOT_PLAN" << endl;
    string channel = "COMMITTED_ROBOT_PLAN";

    int left_arm_control_type = bot_gtk_param_widget_get_enum(pw, PARAM_LEFT_ARM_CT_SELECT);
    int right_arm_control_type = bot_gtk_param_widget_get_enum(pw, PARAM_RIGHT_ARM_CT_SELECT);
    int left_leg_control_type = bot_gtk_param_widget_get_enum(pw, PARAM_LEFT_LEG_CT_SELECT);
    int right_leg_control_type = bot_gtk_param_widget_get_enum(pw, PARAM_RIGHT_LEG_CT_SELECT);
    self->robotPlanListener->commit_compliant_manip_plan(self->robot_utime,channel,left_arm_control_type,right_arm_control_type,left_leg_control_type,right_leg_control_type);
  }
  
  static void spawn_plan_control_type_selection_popup (RendererRobotPlan *self)
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
      pos_x+=0;    pos_y+=100;
      gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
      //gtk_widget_set_size_request (window, 300, 250);
      //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
      gtk_window_set_title(GTK_WINDOW(window), "Select control type");
      gtk_container_set_border_width(GTK_CONTAINER(window), 5);
      pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
      bot_gtk_param_widget_add_separator (pw,"Select Cntl-Type");
      bot_gtk_param_widget_add_enum(pw, PARAM_LEFT_ARM_CT_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 0,"NONE",0,"POSITION",1,"IMPEDANCE", 
                                    2,"STIFF", 3,"COMPLIANT", 4, NULL);
      bot_gtk_param_widget_add_enum(pw, PARAM_RIGHT_ARM_CT_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 0,"NONE",0,"POSITION",1,"IMPEDANCE", 
                                    2,"STIFF", 3,"COMPLIANT", 4, NULL);
      bot_gtk_param_widget_add_enum(pw, PARAM_LEFT_LEG_CT_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 0,"NONE",0,"POSITION",1,"IMPEDANCE", 
                                    2,"STIFF", 3,"COMPLIANT", 4, NULL);
      bot_gtk_param_widget_add_enum(pw, PARAM_RIGHT_LEG_CT_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 0,"NONE",0,"POSITION",1,"IMPEDANCE", 
                                    2,"STIFF", 3,"COMPLIANT", 4, NULL);
      //bot_gtk_param_widget_add_enum(pw, PARAM_LEGS_CT_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 0,"POS_ONLY",0,"LEFT_IMPEDANCE", 
      //                              1,"RIGHT_IMPEDANCE", 2,"BOTH_IMPEDANCE", 3, NULL);
      close_button = gtk_button_new_with_label ("Commit");

      g_signal_connect (G_OBJECT (close_button),
                        "clicked",
                        G_CALLBACK (on_popup_close),
                        (gpointer) window);
      g_signal_connect(G_OBJECT(pw), "destroy",
                       G_CALLBACK(on_control_type_selection_popup_close), self); 

      vbox = gtk_vbox_new (FALSE, 3);
      gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
      gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
      gtk_container_add (GTK_CONTAINER (window), vbox);
      gtk_widget_show_all(window); 
  }   
 //==================================================================================================    
  static gboolean on_stop_walking_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout << "Stopping walking" << endl;
    string channel = "STOP_WALKING";
    self->robotPlanListener->commit_plan_control(self->robot_utime,channel,false,true,false);

    return TRUE;
  }

   static gboolean on_pause_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;

    if(self->robotPlanListener->is_manip_plan()){
      cout <<"Pausing manip plan" << endl;
      string channel = "COMMITTED_PLAN_PAUSE";
      self->robotPlanListener->commit_plan_control(self->robot_utime,channel,true,false,false);
    }

   return TRUE;
  }
  
   static gboolean on_rewind_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;

    if(self->robotPlanListener->is_manip_plan()){
      cout <<"Pausing manip plan and reverting to original pose" << endl;
      string channel = "COMMITTED_PLAN_PAUSE";
      self->robotPlanListener->commit_plan_control(self->robot_utime,channel,false,false,true);
    }

   return TRUE;
  }
   
  static gboolean on_cancel_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    self->robotPlanListener->purge_current_plan();
    
    if(self->robotPlanListener->is_multi_approval_plan()){
      gtk_widget_destroy(self->multiapprove_plan_execution_dock);
      self->multiapprove_plan_execution_dock= NULL;    
    }
    else {
      gtk_widget_destroy(self->plan_execution_dock);
      self->plan_execution_dock= NULL;
    }
    if(self->robotPlanListener->is_walking_plan()) {
      string channel = "STOP_WALKING";
      cout << "Stopping walking" << endl;
      self->robotPlanListener->commit_plan_control(self->robot_utime,channel,false,true,false);
    }
    cout <<"Robot plan terminated" << endl;
    cout <<"Publishing on REJECTED_ROBOT_PLAN" << endl;
    string channel = "REJECTED_ROBOT_PLAN";
    self->robotPlanListener->commit_robot_plan(self->robot_utime,channel);
    
    
   return TRUE;
  }
  
  static  gboolean on_execute_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout <<"Robot plan approved" << endl;
    
        
    if(self->robotPlanListener->is_plan_paused()){
      cout <<"Unpausing manip plan" << endl;
      string channel = "COMMITTED_PLAN_PAUSE";
      self->robotPlanListener->commit_plan_control(self->robot_utime,channel,false,false,false);
      return TRUE;
    }

		if(!self->robotPlanListener->is_walking_plan())
		{
		  string channel;
		  if(self->robotPlanListener->is_manip_plan()){
			  cout <<"Publishing on COMMITTED_ROBOT_PLAN" << endl;
			  channel = "COMMITTED_ROBOT_PLAN";
			  self->robotPlanListener->commit_manip_plan(self->robot_utime,channel);
		  }
		  /*else // only publish a committed robot plan if it is manipulation.
		  {
		    cout <<"Publishing on COMMITTED_ROBOT_PLAN" << endl;
			  channel = "COMMITTED_ROBOT_PLAN";
			  self->robotPlanListener->commit_robot_plan(self->robot_utime,channel);
		  }*/
		  
		}
		else 
		{			
			cout <<"Publishing on COMMITTED_FOOTSTEP_PLAN" << endl;
			string channel = "COMMITTED_FOOTSTEP_PLAN";
			self->robotPlanListener->commit_footstep_plan(self->robot_utime,channel);
		}
		
		if(self->robotPlanListener->is_multi_approval_plan())
		{
		  std::stringstream oss;
		  oss << self->robotPlanListener->_active_breakpoint+1 << "/" << self->robotPlanListener->_num_breakpoints;
		  string str = oss.str();
		  gtk_entry_set_text(GTK_ENTRY(self->breakpoint_entry),str.c_str());   
    	} 		

 // if(!self->robotPlanListener->is_multi_approval_plan()){
 //   gtk_widget_destroy(self->plan_execution_dock);
 //   self->plan_execution_dock= NULL;
 //}
    return TRUE;
  }
  
  static  gboolean on_compliant_execute_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    
//    cout <<"A compliant robot plan approved" << endl;
//    cout <<"Publishing on COMMITTED_COMPLIANT_ROBOT_PLAN" << endl;
//    string channel = "COMMITTED_COMPLIANT_ROBOT_PLAN";
//    self->robotPlanListener->commit_robot_plan(self->robot_utime,channel);
      spawn_plan_control_type_selection_popup(self);
      
 // if(!self->robotPlanListener->is_multi_approval_plan()){
 //   gtk_widget_destroy(self->plan_execution_dock);
 //   self->plan_execution_dock= NULL;
 //}
 
    return TRUE;
  }
   
   
  static void spawn_plan_execution_dock  (void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;

    
    GtkWidget  *execute_button,*compliant_execute_button, *pause_button, *cancel_button, *stop_walking_button, *rewind_button;

    if(self->robotPlanListener->is_multi_approval_plan())
     {
      self->breakpoint_entry = (GtkWidget *) gtk_entry_new();  
      std::stringstream oss;
      oss << self->robotPlanListener->_active_breakpoint+1 << "/" << self->robotPlanListener->_num_breakpoints;
      string str = oss.str();
      gtk_entry_set_text(GTK_ENTRY(self->breakpoint_entry),str.c_str()); 
      gtk_entry_set_width_chars(GTK_ENTRY(self->breakpoint_entry),(gint)4);  
     }

    execute_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PLAY); 
    compliant_execute_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_CONNECT);
    if(self->robotPlanListener->is_manip_plan())
    {
      pause_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PAUSE);
      gtk_widget_set_tooltip_text (pause_button, "Pause");
      rewind_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_REWIND);
      gtk_widget_set_tooltip_text (rewind_button, "Revert (Do Prev QuasiStatic Plan In Reverse)");
    }
    cancel_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_STOP);

    stop_walking_button =  (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_STOP);
    //stop_walking_button = (GtkWidget *) gtk_button_new_with_label(PARAM_STOP_WALKING);

    gtk_widget_set_tooltip_text (execute_button, "Execute Plan");
    gtk_widget_set_tooltip_text (compliant_execute_button, "Execute Plan With Soft Cartesian Compliance");
    gtk_widget_set_tooltip_text (cancel_button, "Cancel Plan");
    gtk_widget_set_tooltip_text(stop_walking_button, "Stop walking NOW");

    
    GtkWidget *hbox;
    hbox = gtk_hbox_new (FALSE, 0);
    GtkToolItem * sep = gtk_separator_tool_item_new ();
    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (sep), FALSE, TRUE,10);
    GtkWidget * label = gtk_label_new ("RobotPlan:");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE,0);    
    gtk_box_pack_start (GTK_BOX (hbox), execute_button, FALSE, FALSE, 3);
    if(self->robotPlanListener->is_multi_approval_plan())
      gtk_box_pack_start (GTK_BOX (hbox), self->breakpoint_entry, FALSE, FALSE,3);
    gtk_box_pack_start (GTK_BOX (hbox), compliant_execute_button, FALSE, FALSE, 3);
    if(self->robotPlanListener->is_manip_plan())   {
      gtk_box_pack_start (GTK_BOX (hbox), pause_button, FALSE, FALSE, 3);
      gtk_box_pack_start (GTK_BOX (hbox), rewind_button, FALSE, FALSE, 3);     
    }   
    if(self->robotPlanListener->is_walking_plan()) {
      gtk_box_pack_start (GTK_BOX (hbox), cancel_button, FALSE, FALSE, 3);
      gtk_box_pack_end(GTK_BOX(hbox), stop_walking_button, FALSE, FALSE, 3);
    } else {
      gtk_box_pack_end (GTK_BOX (hbox), cancel_button, FALSE, FALSE, 3);
    }
      if(self->robotPlanListener->is_manip_plan())   
      gtk_box_pack_start (GTK_BOX (hbox), pause_button, FALSE, FALSE, 3);  
    
    GtkToolItem * toolitem = gtk_tool_item_new ();   
    gtk_container_add (GTK_CONTAINER (toolitem), hbox);   
    gtk_toolbar_insert (GTK_TOOLBAR (self->viewer->toolbar), toolitem, 6);
    if(self->robotPlanListener->is_multi_approval_plan()){    
      if(self->plan_execution_dock!=NULL){ // kill other dock
        gtk_widget_destroy(self->plan_execution_dock);
        self->plan_execution_dock= NULL;  
      } 
      self->multiapprove_plan_execution_dock = GTK_WIDGET(toolitem);
      gtk_widget_show_all (self->multiapprove_plan_execution_dock);  
   }
    else{
      if(self->multiapprove_plan_execution_dock!=NULL){ // kill other dock
        gtk_widget_destroy(self->multiapprove_plan_execution_dock);
        self->multiapprove_plan_execution_dock= NULL;  
      } 
      self->plan_execution_dock = GTK_WIDGET(toolitem);  
      gtk_widget_show_all (self->plan_execution_dock);
    }
    

   g_signal_connect (G_OBJECT (cancel_button),
                  "clicked",
                  G_CALLBACK (on_cancel_button_clicked),
                  self);
   g_signal_connect (G_OBJECT (execute_button),
                  "clicked",
                  G_CALLBACK (on_execute_button_clicked),
                  self);
   g_signal_connect (G_OBJECT (compliant_execute_button),
                  "clicked",
                  G_CALLBACK (on_compliant_execute_button_clicked),
                  self);
   if(self->robotPlanListener->is_manip_plan())   {
     g_signal_connect (G_OBJECT (pause_button),
                  "clicked",
                  G_CALLBACK (on_pause_button_clicked),
                  self);  
     g_signal_connect (G_OBJECT (rewind_button),
              "clicked",
              G_CALLBACK (on_rewind_button_clicked),
              self);                
    } 
    if (self->robotPlanListener->is_walking_plan()) {
      g_signal_connect (G_OBJECT (stop_walking_button),
                        "clicked",
                        G_CALLBACK (on_stop_walking_button_clicked),
                        self);
    }

   /* 
    self->plan_execution_dock = window; 
    gtk_container_add (GTK_CONTAINER (window), hbox);
    gtk_widget_show_all(window);
    gtk_window_set_focus(GTK_WINDOW(window), NULL);*/
    
    gtk_widget_set_can_focus (execute_button,false);
  }
  


  
}
#endif
