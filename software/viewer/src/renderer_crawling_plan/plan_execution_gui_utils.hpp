#ifndef PLAN_EXECUTION_GUI_UTILS_HPP
#define PLAN_EXECUTION_GUI_UTILS_HPP
#define PARAM_LEFT_ARM_CT_SELECT "Left arm  "
#define PARAM_RIGHT_ARM_CT_SELECT "Right arm  "
#define PARAM_LEFT_LEG_CT_SELECT "Left legs  "
#define PARAM_RIGHT_LEG_CT_SELECT "Right legs  "
#define PARAM_STOP_WALKING "Stop crawling"


using namespace renderer_crawling_plan;  
namespace renderer_crawling_plan_gui_utils
{
 //==================================================================================================  
  static gboolean on_popup_close (GtkButton* button, GtkWidget* pWindow)
  {
      gtk_widget_destroy (pWindow);
      return TRUE;
  }


  static gboolean on_stop_walking_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout << "Stopping crawling" << endl;
    string channel = "STOP_CRAWLING";
    self->robotPlanListener->commit_plan_control(self->robot_utime,channel,false,true);

    return TRUE;
  }

   static gboolean on_pause_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;

    if(self->robotPlanListener->is_keyframe_plan()){
      cout <<"Pausing manip plan" << endl;
      string channel = "COMMITTED_PLAN_PAUSE";
      self->robotPlanListener->commit_plan_control(self->robot_utime,channel,true,false);
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

    string channel = "STOP_CRAWLING";
    cout << "Stopping crawling" << endl;
    self->robotPlanListener->commit_plan_control(self->robot_utime,channel,false,true);

   return TRUE;
  }
  
  static  gboolean on_execute_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout <<"Crawling plan approved" << endl;
    
        
    if(self->robotPlanListener->is_plan_paused()){
      cout <<"Unpausing manip plan" << endl;
      string channel = "COMMITTED_PLAN_PAUSE";
      self->robotPlanListener->commit_plan_control(self->robot_utime,channel,false,false);
      return TRUE;
    }

    cout <<"Publishing on COMMITTED_CRAWLING_GOAL" << endl;
    string channel = "COMMITTED_CRAWLING_GOAL";
    self->robotPlanListener->commit_walking_goal(self->robot_utime,channel);

		
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
  
   
   
  static void spawn_plan_execution_dock  (void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;

    
    GtkWidget  *execute_button,*compliant_execute_button, *pause_button, *cancel_button, *stop_walking_button;

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
    /* compliant_execute_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_CONNECT);
    if(self->robotPlanListener->is_keyframe_plan())
    {
      pause_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PAUSE);
      gtk_widget_set_tooltip_text (pause_button, "Pause");
    }*/
    cancel_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_STOP);

    stop_walking_button =  (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_STOP);
    //stop_walking_button = (GtkWidget *) gtk_button_new_with_label(PARAM_STOP_WALKING);

    gtk_widget_set_tooltip_text (execute_button, "Execute Plan");
    //gtk_widget_set_tooltip_text (compliant_execute_button, "Execute Plan With Soft Cartesian Compliance");
    gtk_widget_set_tooltip_text (cancel_button, "Cancel Plan");
    gtk_widget_set_tooltip_text(stop_walking_button, "Stop walking NOW");

    
    GtkWidget *hbox;
    hbox = gtk_hbox_new (FALSE, 0);
    GtkToolItem * sep = gtk_separator_tool_item_new ();
    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (sep), FALSE, TRUE,10);
    GtkWidget * label = gtk_label_new ("CrawlingPlan:");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE,0);    
    gtk_box_pack_start (GTK_BOX (hbox), execute_button, FALSE, FALSE, 3);
    if(self->robotPlanListener->is_multi_approval_plan())
      gtk_box_pack_start (GTK_BOX (hbox), self->breakpoint_entry, FALSE, FALSE,3);
    gtk_box_pack_start (GTK_BOX (hbox), compliant_execute_button, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (hbox), cancel_button, FALSE, FALSE, 3);
    gtk_box_pack_end(GTK_BOX(hbox), stop_walking_button, FALSE, FALSE, 3);
    
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
  g_signal_connect (G_OBJECT (stop_walking_button),
                    "clicked",
                    G_CALLBACK (on_stop_walking_button_clicked),
                    self);


    gtk_widget_set_can_focus (execute_button,false);
  }
  


  
}
#endif
