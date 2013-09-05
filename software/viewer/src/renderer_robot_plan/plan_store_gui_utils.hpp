#ifndef PLAN_STORE_GUI_UTILS_HPP
#define PLAN_STORE_GUI_UTILS_HPP
#define PARAM_STORE "Store"
#define PARAM_CLEAR "Clear old"
#define PARAM_PLAN_NAME "Name:"

#include "renderer_robot_plan.hpp"

using namespace std;
using namespace boost;
using namespace renderer_robot_plan;
namespace renderer_robot_plan_gui_utils
{
   
  static void on_afftriggered_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user; 
    self->afftriggered_popup = NULL;
  }
  
  static void on_plan_storage_popup_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererRobotPlan *self = (RendererRobotPlan*) user;
    std::string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; 
    std::string otdf_filepath,plan_xml_dirpath;
    otdf_filepath =  otdf_models_path + (*self->trigger_source_otdf_id) +".otdf";
    plan_xml_dirpath =  otdf_models_path + "stored_plans/";
                
    if ((!strcmp(name, PARAM_STORE))) {
   
        std::time_t rawtime;
        std::tm* timeinfo;
        char buffer [80];
        std::time(&rawtime);
        timeinfo = std::localtime(&rawtime);
        std::strftime(buffer,80,"%Y-%m-%d-%H-%M::",timeinfo);
        std::string timestamp_str = buffer;
        std::string plan_name = bot_gtk_param_widget_get_text_entry(pw,PARAM_PLAN_NAME);

        PlanSeed planSeed;
        //plan_ref needs to be unique timestamp::Name.
        planSeed.plan_ref = timestamp_str+plan_name;
 
          
        if(self->robotPlanListener->_gl_robot_list.size()>0) // 
        {
         self->robotPlanListener->prepareActivePlanForStorage(self->T_world_trigger_aff, 
                                  planSeed.plan_type,
                                  planSeed.stateframe_ids,
                                  planSeed.stateframe_values,
                                  planSeed.graspframe_ids,
                                  planSeed.graspframe_values);
          std::cout << planSeed.plan_ref << std::endl;
          planSeed.writePlanToXMLFile((*self->trigger_source_otdf_id),plan_xml_dirpath);
          // cross ref in OTDF
          planSeed.writeToOtdf(otdf_filepath);
        }
        else
           cout <<"No Active Plan To Store \n";

    }
    /*
    else if ((!strcmp(name, PARAM_UNSTORE))) {
    }
    else if ((!strcmp(name, PARAM_CLEAR))) {
      PlanSeed planSeed;
      planSeed.clearAllFromOtdf(otdf_filepath,plan_xml_dirpath);
    }*/

    gtk_widget_destroy(self->afftriggered_popup);
  }
  
  static void spawn_plan_storage_addition_popup (void *user)
  {
  
   RendererRobotPlan *self = (RendererRobotPlan*) user;

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
    gtk_window_set_title(GTK_WINDOW(window), "store plan");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    std::stringstream oss;
    oss << bot_timestamp_now();
    std::string t_str = oss.str();
    
    bot_gtk_param_widget_add_text_entry(pw,PARAM_PLAN_NAME,BOT_GTK_PARAM_WIDGET_ENTRY,"");
    bot_gtk_param_widget_add_buttons(pw,PARAM_STORE,NULL);
    //bot_gtk_param_widget_add_buttons(pw,PARAM_CLEAR,NULL);
   
    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_plan_storage_popup_param_widget_changed), self);
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
