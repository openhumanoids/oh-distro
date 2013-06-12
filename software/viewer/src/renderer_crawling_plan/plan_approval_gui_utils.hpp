#ifndef PLAN_APPROVAL_GUI_UTILS_HPP
#define PLAN_APPROVAL_GUI_UTILS_HPP

#include "renderer_crawling_plan.hpp"

using namespace std;
using namespace boost;
using namespace renderer_crawling_plan;
namespace renderer_crawling_plan_gui_utils
{
   
  static gboolean on_plan_cancel_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout <<"Publishing on REJECTED_MANIP_MAP" << endl;
    string channel = "REJECTED_MANIP_MAP";   
    self->robotPlanListener->commit_manip_map(self->robot_utime,channel);    
    self->robotPlanListener->purge_current_plan();
    gtk_widget_destroy (self->plan_approval_dock);
    self->plan_approval_dock= NULL;
    bot_viewer_request_redraw(self->viewer);
   return TRUE;
  }
  
  static  gboolean on_plan_approve_button_clicked (GtkButton* button, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    cout <<"Manip Map approved" << endl;
    cout <<"Publishing on COMMITTED_MANIP_MAP" << endl;
    string channel = "COMMITTED_MANIP_MAP";
    self->robotPlanListener->commit_manip_map(self->robot_utime,channel);   
 
    //gtk_widget_destroy (self->plan_approval_dock);
    //self->plan_approval_dock= NULL;
    // self->robotPlanListener->_waiting_for_new_plan = true;
    bot_viewer_request_redraw(self->viewer);
    return TRUE;
  }
   
  static void spawn_plan_approval_dock (void *user)
  {
  
   RendererRobotPlan *self = (RendererRobotPlan*) user;

    GtkWidget  *approve_button, *cancel_button;
    approve_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_APPLY);
    cancel_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_CANCEL);

        
    gtk_widget_set_tooltip_text (approve_button, "Accept Manip Map");
    gtk_widget_set_tooltip_text (cancel_button, "Cancel Manip Map");
   
    
   //self->plan_approval_dock = window;    
   g_signal_connect (G_OBJECT (cancel_button),
                  "clicked",
                  G_CALLBACK (on_plan_cancel_button_clicked),
                  self);

   g_signal_connect (G_OBJECT (approve_button),
                  "clicked",
                  G_CALLBACK (on_plan_approve_button_clicked),
                  self);
    GtkWidget *hbox;
    hbox = gtk_hbox_new (FALSE, 0);
    //GtkWidget * sep = gtk_vseparator_new ();  
    GtkToolItem * sep = gtk_separator_tool_item_new ();
    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (sep), FALSE, TRUE,10);
    GtkWidget * label = gtk_label_new ("Manip Map:");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE,0);
    gtk_box_pack_start (GTK_BOX (hbox), approve_button, FALSE, FALSE,0);
    gtk_box_pack_start (GTK_BOX (hbox), cancel_button, FALSE, FALSE,0);


  GtkToolItem * toolitem = gtk_tool_item_new ();   
  gtk_container_add (GTK_CONTAINER (toolitem), hbox);   
  gtk_toolbar_insert (GTK_TOOLBAR (self->viewer->toolbar), toolitem, 6);
  self->plan_approval_dock = GTK_WIDGET(toolitem);  
  std::cout << "running gtk_widget_show_all in plan approval gui spawn\n";
  gtk_widget_show_all (self->plan_approval_dock);

  }

  
  
  
}
#endif
