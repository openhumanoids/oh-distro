#ifndef POSE_APPROVAL_GUI_UTILS_HPP
#define POSE_APPROVAL_GUI_UTILS_HPP

#include "renderer_robot_state.hpp"

using namespace std;
using namespace boost;
using namespace renderer_robot_state;
namespace renderer_robot_state_gui_utils
{
   
  static gboolean on_pose_cancel_button_clicked (GtkButton* button, void *user)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    if(self->robotStateListener->_gl_robot->is_future_state_changing())
      self->robotStateListener->_gl_robot->set_future_state_changing(false);
    self->robotStateListener->_gl_robot->set_future_state( self->robotStateListener->_gl_robot->_T_world_body, self->robotStateListener->_gl_robot->_current_jointpos);   
    self->robotStateListener->_gl_robot->disable_future_display();   
    gtk_widget_destroy (self->pose_approval_dock);
    self->pose_approval_dock= NULL;
    bot_viewer_request_redraw(self->viewer);
   return TRUE;
  }
  
  static  gboolean on_pose_approve_button_clicked (GtkButton* button, void *user)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    cout <<"Publishing a WALKING_GOAL to desired end pose" << endl;
    string channel = "WALKING_GOAL";
    publish_walking_goal(self,channel); 
 
    gtk_widget_destroy (self->pose_approval_dock);
    self->pose_approval_dock= NULL;
    bot_viewer_request_redraw(self->viewer);
    return TRUE;
  }
   
  static void spawn_pose_approval_dock (void *user)
  {
  
   RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
   
   if(self->pose_approval_dock!=NULL){
      gtk_widget_destroy (self->pose_approval_dock);
      self->pose_approval_dock=NULL;
    }
    GtkWidget  *approve_button, *cancel_button;
    approve_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_APPLY);
    cancel_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_CANCEL);

        
    gtk_widget_set_tooltip_text (approve_button, "Accept Robot End Pose");
    gtk_widget_set_tooltip_text (cancel_button, "Cancel Robot End Pose");
   
    
    //self->pose_approval_dock = window;    
    g_signal_connect (G_OBJECT (cancel_button),
                  "clicked",
                  G_CALLBACK (on_pose_cancel_button_clicked),
                  self);

    g_signal_connect (G_OBJECT (approve_button),
                  "clicked",
                  G_CALLBACK (on_pose_approve_button_clicked),
                  self);
    GtkWidget *hbox;
    hbox = gtk_hbox_new (FALSE, 0);
    //GtkWidget * sep = gtk_vseparator_new ();  
    GtkToolItem * sep = gtk_separator_tool_item_new ();
    gtk_box_pack_start (GTK_BOX (hbox), GTK_WIDGET (sep), FALSE, TRUE,10);
    GtkWidget * label = gtk_label_new ("Robot EndPose:");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE,0);
    gtk_box_pack_start (GTK_BOX (hbox), approve_button, FALSE, FALSE,0);
    gtk_box_pack_start (GTK_BOX (hbox), cancel_button, FALSE, FALSE,0);


    GtkToolItem * toolitem = gtk_tool_item_new ();   
    gtk_container_add (GTK_CONTAINER (toolitem), hbox);   
    gtk_toolbar_insert (GTK_TOOLBAR (self->viewer->toolbar), toolitem, 6);
    self->pose_approval_dock = GTK_WIDGET(toolitem); 
    gtk_widget_show_all (self->pose_approval_dock);

  }

  
  
  
}
#endif
