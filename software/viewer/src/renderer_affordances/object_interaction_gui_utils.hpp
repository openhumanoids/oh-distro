#ifndef OBJECT_INTERACTION_GUI_UTILS_HPP
#define OBJECT_INTERACTION_GUI_UTILS_HPP

#define PARAM_SEED_LH "Seed LHand"
#define PARAM_SEED_RH "Seed RHand"
#define PARAM_SEED_LF "Seed LFoot"
#define PARAM_SEED_RF "Seed RFoot"
#define PARAM_ENABLE_ADJUSTMENT "Enable Adjustment"

using namespace renderer_affordances;

namespace renderer_affordances_gui_utils
{

//  OTDF object rightclk popup and associated cbs.
  static void on_object_geometry_rtclk_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
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
    
    bot_viewer_request_redraw(self->viewer);
    gtk_widget_destroy(self->rtclk_popup);
  }
  static void on_object_geometry_rtclk_popup_close (BotGtkParamWidget *pw, void *user)
  {
    // TODO: Send publish affordance command msg
  }

  static void spawn_object_geometry_rtclk_popup (RendererAffordances *self)
  {

    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw;

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 200, 250);
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "Rtclk");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_LH, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_RH, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_LF, NULL);
    bot_gtk_param_widget_add_buttons(pw,PARAM_SEED_RF, NULL);
    
         typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
     object_instance_map_type_::iterator it= self->instantiated_objects.find((*self->object_selection));
     bool val;
     if(it!=self->instantiated_objects.end())
      val = it->second._gl_object->is_link_adjustment_enabled();
     else
      val =false;
    bot_gtk_param_widget_add_booleans(pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_ENABLE_ADJUSTMENT, val, NULL);
    //cout <<*self->selection << endl; // otdf_type::geom_name
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_object_geometry_rtclk_popup_param_widget_changed), self);

    self->rtclk_popup  = window;

    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),"clicked",G_CALLBACK (on_popup_close),(gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy", G_CALLBACK(on_object_geometry_rtclk_popup_close), self); 


    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
      gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 
  }

}
#endif
