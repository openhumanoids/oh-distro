#ifndef PLAN_EXECUTION_GUI_UTILS_HPP
#define PLAN_EXECUTION_GUI_UTILS_HPP

#define PARAM_STOP_WALKING "Stop walking"


using namespace renderer_sticky_feet;  
namespace renderer_sticky_feet_gui_utils
{
 //==================================================================================================  
  static gboolean on_popup_close (GtkButton* button, GtkWidget* pWindow)
  {
      gtk_widget_destroy (pWindow);
      return TRUE;
  }
  //==================================================================================================    
  static gboolean on_bdiplan_cancel_button_clicked (GtkButton* button, void *user)
  {
    RendererStickyFeet *self = (RendererStickyFeet*) user;
    cout <<"Publishing on REJECTED_FOOTSTEP_PLAN" << endl;
    string channel = "REJECTED_FOOTSTEP_PLAN";   
    self->footStepPlanListener->commit_footstep_plan(self->robot_utime,channel);    
    self->footStepPlanListener->_gl_planned_stickyfeet_list.clear();    
    gtk_widget_destroy (self->plan_execution_dock);
    self->plan_execution_dock= NULL;
    self->plan_execute_button = NULL;
    self->footStepPlanListener->_waiting_for_new_plan = true;
    bot_viewer_request_redraw(self->viewer);
   return TRUE;
  }
 //==================================================================================================    
  static gboolean on_bdiplan_stop_walking_button_clicked (GtkButton* button, void *user)
  {
    RendererStickyFeet *self = (RendererStickyFeet*) user;
    cout << "Stopping walking" << endl;
    string channel = "STOP_WALKING";
    self->footStepPlanListener->commit_plan_control(self->robot_utime,channel,false,true,false);
    return TRUE;
  }

  
  static  gboolean on_bdiplan_execute_button_clicked (GtkButton* button, void *user)
  {
    RendererStickyFeet *self = (RendererStickyFeet*) user;
      cout <<"BDI foot step plan committed" << endl;
			cout <<"Publishing on COMMITTED_FOOTSTEP_PLAN" << endl;
			string channel = "COMMITTED_FOOTSTEP_PLAN";
      self->footStepPlanListener->commit_footstep_plan(self->robot_utime,channel);  
      
      drc::utime_t msg;
      msg.utime =bot_timestamp_now();
      // All other active viewers will also kill their respective execute buttons
      self->lcm->publish("FOOTSTEP_PLAN_EXECUTE_EVENT", &msg); 
      self->footStepPlanListener->_last_plan_approved_or_executed = true;
      gtk_widget_destroy (self->plan_execute_button);
      self->plan_execute_button= NULL;
		
    return TRUE;
  }
  
   
  static void spawn_plan_execution_dock  (void *user)
  {
    RendererStickyFeet *self = (RendererStickyFeet*) user;

    
    GtkWidget  *stop_walking_button,*cancel_button;

    self->plan_execute_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PLAY); 
    stop_walking_button =  (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_STOP);
    cancel_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_CANCEL);

    gtk_widget_set_tooltip_text (self->plan_execute_button, "Execute BDI FootStep Plan");
    gtk_widget_set_tooltip_text(stop_walking_button, "Stop walking NOW");
    gtk_widget_set_tooltip_text (cancel_button, "Clear Plan Cache");
    
    GtkWidget *hbox;
    hbox = gtk_hbox_new (FALSE, 0);
    GtkToolItem * sep = gtk_separator_tool_item_new ();
    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (sep), FALSE, TRUE,10);
    GtkWidget * label = gtk_label_new ("FootStepPlan:");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE,0);    
    gtk_box_pack_start (GTK_BOX (hbox), self->plan_execute_button, FALSE, FALSE, 3);
    gtk_box_pack_end(GTK_BOX(hbox), stop_walking_button, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (hbox), cancel_button, FALSE, FALSE, 3);
    
    GtkToolItem * toolitem = gtk_tool_item_new ();   
    gtk_container_add (GTK_CONTAINER (toolitem), hbox);   
    gtk_toolbar_insert (GTK_TOOLBAR (self->viewer->toolbar), toolitem, 6);
   
    g_signal_connect (G_OBJECT (self->plan_execute_button),
                  "clicked",
                  G_CALLBACK (on_bdiplan_execute_button_clicked),
                  self);
    g_signal_connect (G_OBJECT (stop_walking_button),
                      "clicked",
                      G_CALLBACK (on_bdiplan_stop_walking_button_clicked),
                      self);
   g_signal_connect (G_OBJECT (cancel_button),
                  "clicked",
                  G_CALLBACK (on_bdiplan_cancel_button_clicked),
                  self);
                      
    gtk_widget_set_can_focus (self->plan_execute_button,false);
    self->plan_execution_dock = GTK_WIDGET(toolitem);  
      gtk_widget_show_all (self->plan_execution_dock);
  }
  


  
}
#endif
