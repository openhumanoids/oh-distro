#ifndef PLAN_EXECUTION_GUI_UTILS_HPP
#define PLAN_EXECUTION_GUI_UTILS_HPP

using namespace renderer_robot_plan;  
namespace renderer_robot_plan_gui_utils
{
   
  static gboolean on_cancel_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    self->robotPlanListener->_gl_robot_list.clear();
    if(self->robotPlanListener->_is_manip_plan){
     self->robotPlanListener->_gl_robot_keyframe_list.clear();
     self->robotPlanListener->_gl_left_hand->enable_bodypose_adjustment(false); 
     self->robotPlanListener->_gl_right_hand->enable_bodypose_adjustment(false);
     self->robotPlanListener->_gl_left_foot->enable_bodypose_adjustment(false); 
     self->robotPlanListener->_gl_right_foot->enable_bodypose_adjustment(false);
     self->robotPlanListener->_is_manip_plan = false;
    }
    gtk_widget_destroy(self->plan_execution_dock);
    self->plan_execution_dock= NULL;
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
    cout <<"Publishing on COMMITTED_ROBOT_PLAN" << endl;
    string channel = "COMMITTED_ROBOT_PLAN";
    self->robotPlanListener->commit_robot_plan(self->robot_utime,channel);
 //   gtk_widget_destroy(self->plan_execution_dock);
 //   self->plan_execution_dock= NULL;
    return TRUE;
  }
  
  static  gboolean on_compliant_execute_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout <<"A compliant robot plan approved" << endl;
    cout <<"Publishing on COMMITTED_COMPLIANT_ROBOT_PLAN" << endl;
    string channel = "COMMITTED_COMPLIANT_ROBOT_PLAN";
    self->robotPlanListener->commit_robot_plan(self->robot_utime,channel);
 //   gtk_widget_destroy(self->plan_execution_dock);
 //   self->plan_execution_dock= NULL;
    return TRUE;
  }
   
   
  static void spawn_plan_execution_dock  (void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;

    
/*    GtkWidget *window;
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    //gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_NONE);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 150, 50);
    gint pos_x, pos_y;
//    gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
//    pos_x-=75;    pos_y+=75;
    gint root_x, root_y;
    gtk_window_get_position (GTK_WINDOW(self->viewer->window), &root_x, &root_y);
    
    gint width, height;
    gtk_window_get_size(GTK_WINDOW(self->viewer->window),&width,&height);
    
    pos_x=root_x+0.5*width;    pos_y=root_y+0.75*height;
   // gint gdk_screen_height  (void);//Returns the height of the default screen in pixels.
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);


   // gint gdk_screen_height  (void);//Returns the height of the default screen in pixels.
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
    
    gtk_window_set_title(GTK_WINDOW(window), "Plan Execution Control Dock");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
*/
    
    GtkWidget  *execute_button,*compliant_execute_button, *pause_button, *cancel_button;
    execute_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PLAY);
    compliant_execute_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_CONNECT);
    pause_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PAUSE);
    cancel_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_STOP);
        
    gtk_widget_set_tooltip_text (execute_button, "Execute Plan");
    gtk_widget_set_tooltip_text (compliant_execute_button, "Execute Plan With Soft Cartesian Compliance");
    gtk_widget_set_tooltip_text (pause_button, "Pause (To Be Implemented)");
    gtk_widget_set_tooltip_text (cancel_button, "Cancel Plan");
    
    /*GtkWidget  *grasp_button, *ungrasp_button;
     grasp_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_CONNECT);
     ungrasp_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_DISCONNECT);

    gtk_widget_set_tooltip_text (grasp_button, "grasp");
    gtk_widget_set_tooltip_text (ungrasp_button, "Ungrasp");*/
 
  // none of these work
    /*gtk_widget_set_can_focus (play_button,false);
    gtk_window_set_focus(GTK_WINDOW(window), NULL);
    gtk_widget_set_can_focus (pause_button,FALSE);
    gtk_widget_set_can_focus (close_button,FALSE);*/
    //gtk_widget_grab_focus (GTK_WINDOW(self->viewer->window));

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
                     
     
    GtkWidget *hbox;
 hbox = gtk_hbox_new (FALSE, 0);
    //GtkWidget * sep = gtk_vseparator_new ();  
    GtkToolItem * sep = gtk_separator_tool_item_new ();
    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (sep), FALSE, TRUE,10);
    GtkWidget * label = gtk_label_new ("RobotPlan:");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE,0);
    gtk_box_pack_start (GTK_BOX (hbox), execute_button, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (hbox), compliant_execute_button, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (hbox), pause_button, FALSE, FALSE, 3);
    gtk_box_pack_end (GTK_BOX (hbox), cancel_button, FALSE, FALSE, 3);
    
    
    GtkToolItem * toolitem = gtk_tool_item_new ();   
    gtk_container_add (GTK_CONTAINER (toolitem), hbox);   
    gtk_toolbar_insert (GTK_TOOLBAR (self->viewer->toolbar), toolitem, 6);
    self->plan_execution_dock = GTK_WIDGET(toolitem);  
    gtk_widget_show_all (self->plan_execution_dock);


   /* 
   

   self->plan_execution_dock = window; 
   gtk_container_add (GTK_CONTAINER (window), hbox);
    gtk_widget_show_all(window);
    gtk_window_set_focus(GTK_WINDOW(window), NULL);*/
    
    gtk_widget_set_can_focus (execute_button,false);
  }

  
  
  
}
#endif
