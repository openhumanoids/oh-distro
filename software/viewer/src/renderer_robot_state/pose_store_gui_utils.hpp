#ifndef POSE_STORE_GUI_UTILS_HPP
#define POSE_STORE_GUI_UTILS_HPP
#define PARAM_STORE "Store"
#define PARAM_CLEAR "Clear old"
#define PARAM_POSE_NAME "Name:"

#include "renderer_robot_state.hpp"

using namespace std;
using namespace boost;
using namespace renderer_robot_state;
namespace renderer_robot_state_gui_utils
{
   
  static void on_afftriggered_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user; 
    self->afftriggered_popup = NULL;
  }
  
  static void on_pose_storage_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
    std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
    std::string otdf_filepath,pose_xml_dirpath;
    otdf_filepath =  otdf_models_path + (*self->trigger_source_otdf_id) +".otdf";
    pose_xml_dirpath =  otdf_models_path + "stored_poses/";
                
    if ((!strcmp(name, PARAM_STORE))) {
   
        std::time_t rawtime;
        std::tm* timeinfo;
        char buffer [80];
        std::time(&rawtime);
        timeinfo = std::localtime(&rawtime);
        std::strftime(buffer,80,"%Y-%m-%d-%H-%M::",timeinfo);
        std::string timestamp_str = buffer;
        std::string pose_name = bot_gtk_param_widget_get_text_entry(pw,PARAM_POSE_NAME);

        PoseSeed poseSeed;
        //pose_ref needs to be unique timestamp::Name.
        poseSeed.pose_ref = timestamp_str+pose_name;

        if(self->robotStateListener->_gl_robot->is_future_display_active())   
        {
        
          //type = "endpose";
          visualization_utils::prepareEndPoseForStorage(self->T_world_trigger_aff,
                                                        self->robotStateListener->_received_endpose,
                                                        poseSeed.stateframe_ids,
                                                        poseSeed.stateframe_values);
        /* self->robotStateListener->prepareDesiredRobotStateForEndPoseStorage(self->T_world_trigger_aff, 
                                  poseSeed.pose_type,
                                  poseSeed.stateframe_ids,
                                  poseSeed.stateframe_values,
                                  poseSeed.graspframe_ids,
                                  poseSeed.graspframe_values);*/
          std::cout << poseSeed.pose_ref << std::endl;
          poseSeed.writePoseToXMLFile((*self->trigger_source_otdf_id),pose_xml_dirpath);
          // cross ref in OTDF
          poseSeed.writeToOtdf(otdf_filepath);
        }
        else
           cout <<"No Active Plan To Store \n";

    }
    /*
    else if ((!strcmp(name, PARAM_UNSTORE))) {
    }
    else if ((!strcmp(name, PARAM_CLEAR))) {
      PoseSeed poseSeed;
      poseSeed.clearAllFromOtdf(otdf_filepath,pose_xml_dirpath);
    }*/

    gtk_widget_destroy(self->afftriggered_popup);
  }
  
  static void spawn_endpose_storage_addition_popup (void *user)
  {
  
   RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;

    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw;

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);//GTK_WIN_POS_CENTER/MOUSE
    gtk_window_set_default_size(GTK_WINDOW(window), 150, 50);
    gint pos_x, pos_y;
    gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
    pos_x+=0;    pos_y-=0;
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
    gtk_window_set_title(GTK_WINDOW(window), "store pose");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    std::stringstream oss;
    oss << bot_timestamp_now();
    std::string t_str = oss.str();
    
    bot_gtk_param_widget_add_text_entry(pw,PARAM_POSE_NAME,BOT_GTK_PARAM_WIDGET_ENTRY,"");
    bot_gtk_param_widget_add_buttons(pw,PARAM_STORE,NULL);
    //bot_gtk_param_widget_add_buttons(pw,PARAM_CLEAR,NULL);
   
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_pose_storage_popup_param_widget_changed), self);
    self->afftriggered_popup  = window;

    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),"clicked",G_CALLBACK (on_popup_close),(gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy", G_CALLBACK(on_afftriggered_popup_close), self); 


    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 
  }
  
  
  
}
#endif
