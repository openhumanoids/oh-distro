#ifndef MIXEDSEED_INTERACTION_GUI_UTILS_HPP
#define MIXEDSEED_INTERACTION_GUI_UTILS_HPP

#include "object_interaction_gui_utils.hpp"

using namespace renderer_affordances;
using namespace renderer_affordances_lcm_utils;

namespace renderer_affordances_gui_utils
{
  //--------------------------------------------------------------------------
  // Mixed seed Interaction
  //
  static void on_mixed_seed_dblclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
      
    if(self->seedSelectionManager->get_selection_cnt()==0){
        gtk_widget_destroy(self->dblclk_popup);
        return;
    }
      
    if (! strcmp(name, PARAM_DELETE)) {
      cout << "\n Clearing selected seeds\n";
      self->stickyHandCollection->remove_selected(self->seedSelectionManager);
      self->stickyFootCollection->remove_selected(self->seedSelectionManager);     
      bot_viewer_request_redraw(self->viewer);
    }
    else if ((!strcmp(name, PARAM_STORE))) {
      self->stickyHandCollection->store_selected(self->seedSelectionManager,false,self->affCollection);
      self->stickyFootCollection->store_selected(self->seedSelectionManager,false,self->affCollection);
    }
    else if ((!strcmp(name, PARAM_UNSTORE))) {
      self->stickyHandCollection->store_selected(self->seedSelectionManager,true,self->affCollection);
      self->stickyFootCollection->store_selected(self->seedSelectionManager,true,self->affCollection);
    }
    else if(!strcmp(name,PARAM_SEND_EE_GOAL_SEQUENCE)){
      string channel = "DESIRED_WHOLE_BODY_PLAN_EE_GOAL_SEQUENCE";
      cout << "publishing EE goal sequence on " << channel << endl;
      publish_EE_goal_sequence_and_get_whole_body_plan(self,channel,false);  
    }
      bot_viewer_request_redraw(self->viewer);
      gtk_widget_destroy(self->dblclk_popup);
    
  }
    
  static void spawn_mixed_seed_dblclk_popup (void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
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
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEND_EE_GOAL_SEQUENCE, NULL); 
    bot_gtk_param_widget_add_buttons(pw,PARAM_STORE, NULL); 
    bot_gtk_param_widget_add_buttons(pw,PARAM_UNSTORE, NULL); 

    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_mixed_seed_dblclk_popup_param_widget_changed), self);
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
