#ifndef POPUP_GUI_UTILS_HPP
#define POPUP_GUI_UTILS_HPP

#include "renderer_robot_state.hpp"
#include "RobotStateListener.hpp"
#define PARAM_SELECT_EE_TYPE "EE :"
#define PARAM_SELECT_RES  "Res (m): "
#define PARAM_SELECT_ANG_RES "AngRes (deg): "
using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;
using namespace renderer_robot_state;

namespace renderer_robot_state_gui_utils
{   
  static void publish_teleop_cmd( Eigen::Vector3f dir, void* user, bool is_rotation_adjustment)
  {

    RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
   drc::ee_cartesian_adjust_t msg;
   msg.utime = bot_timestamp_now();
   if(self->active_ee == 0)
     msg.ee_type =msg.LEFT_HAND;
   else if(self->active_ee == 1)
     msg.ee_type =msg.RIGHT_HAND; 
   
   if(!is_rotation_adjustment)  {
   msg.pos_delta.x=dir[0]*self->active_res;
   msg.pos_delta.y=dir[1]*self->active_res;
   msg.pos_delta.z=dir[2]*self->active_res;      
   msg.rpy_delta.x=0;
   msg.rpy_delta.y=0;
   msg.rpy_delta.z=0;  
   }
   else
   {
    msg.pos_delta.x=0;
    msg.pos_delta.y=0;
    msg.pos_delta.z=0;  
    msg.rpy_delta.x=dir[0]*self->active_angres*(M_PI/180);
    msg.rpy_delta.y=dir[1]*self->active_angres*(M_PI/180);
    msg.rpy_delta.z=dir[2]*self->active_angres*(M_PI/180);
   }
    //self->lcm->publish("COMMITTED_EE_ADJUSTMENT", &msg);
    self->lcm->publish("CANDIDATE_EE_ADJUSTMENT", &msg);
  }  
  static gboolean on_popup_close (GtkButton* button, GtkWidget* pWindow)
  {
      gtk_widget_destroy (pWindow);
      return TRUE;
  }
    
  static void on_teleop_button_clicked(GtkToolButton* button, RobotStateRendererStruc *self)
  {
   
    Eigen::Vector3f dir;
    dir << 0,0,0;
    std::string id(gtk_tool_button_get_stock_id (button));
    if(id=="gtk-go-up") {
      std::stringstream oss;
      oss << "+z:" << self->active_res << " m";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,0,1;
    }
    else if (id=="gtk-go-down") { 
      std::stringstream oss;
      oss << "-z:" << self->active_res << " m";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,0,-1;
     }
    else if (id=="gtk-go-forward") {
      std::stringstream oss;
      oss << "+x:" << self->active_res << " m";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 1,0,0;
    }
    else if (id=="gtk-go-back") {
      std::stringstream oss;
      oss << "-x:" << self->active_res << " m";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << -1,0,0;
    }
      
     publish_teleop_cmd(dir,self,false);  
      
  }
  
  
  static void on_y_teleop_button_clicked(GtkToolButton* button, RobotStateRendererStruc *self)
  {
   
    Eigen::Vector3f dir;
    dir << 0,0,0;
    std::string id(gtk_tool_button_get_stock_id (button));
    if(id=="gtk-go-up") {
      std::stringstream oss;
      oss << "+y:" << self->active_res << " m";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,1,0;
    }
    else if (id=="gtk-go-down") { 
      std::stringstream oss;
      oss << "-y:" << self->active_res << " m";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,-1,0;
     }
      
     publish_teleop_cmd(dir,self,false);  
  }  
  
   static void on_r_teleop_button_clicked(GtkToolButton* button, RobotStateRendererStruc *self)
  {
   
    Eigen::Vector3f dir;
    dir << 0,0,0;
    std::string id(gtk_tool_button_get_stock_id (button));
    if(id=="gtk-go-up") {
      std::stringstream oss;
      oss << "+Rol:" << self->active_angres << " deg";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 1,0,0;
    }
    else if (id=="gtk-go-down") { 
      std::stringstream oss;
      oss << "-Rol:" << self->active_angres << " deg";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << -1,0,0;
     }
     publish_teleop_cmd(dir,self,true);  
  } 
  
  static void on_p_teleop_button_clicked(GtkToolButton* button, RobotStateRendererStruc *self)
  {
   
    Eigen::Vector3f dir;
    dir << 0,0,0;
    std::string id(gtk_tool_button_get_stock_id (button));
    if(id=="gtk-go-up") {
      std::stringstream oss;
      oss << "+Pit:" << self->active_angres << " deg";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,1,0;
    }
    else if (id=="gtk-go-down") { 
      std::stringstream oss;
      oss << "-Pit:" << self->active_angres << " deg";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,-1,0;
     }
     publish_teleop_cmd(dir,self,true);  
  }  
  
  static void on_yaw_teleop_button_clicked(GtkToolButton* button, RobotStateRendererStruc *self)
  {
   
    Eigen::Vector3f dir;
    dir << 0,0,0;
    std::string id(gtk_tool_button_get_stock_id (button));
    if(id=="gtk-go-up") {
      std::stringstream oss;
      oss << "Yaw:" << self->active_angres << " deg";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,0,1;
    }
    else if (id=="gtk-go-down") { 
      std::stringstream oss;
      oss << "-Yaw:" << self->active_angres << " deg";
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),(oss.str()).c_str()); 
      dir << 0,0,-1;
     }
     publish_teleop_cmd(dir,self,true);  
  }   
  
   static void on_teleop_ee_popup_close (BotGtkParamWidget *pw, void *user)
    {

        RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;        
        self->teleop_popup = NULL;
        bot_viewer_request_redraw(self->viewer);
    }  

    static void on_teleop_ee_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
    {
        RobotStateRendererStruc *self = (RobotStateRendererStruc*) user;
        self->active_ee = bot_gtk_param_widget_get_enum(pw,PARAM_SELECT_EE_TYPE);
        self->active_res = bot_gtk_param_widget_get_double(pw,PARAM_SELECT_RES);
        self->active_angres = bot_gtk_param_widget_get_double(pw,PARAM_SELECT_ANG_RES);
    }

    static void spawn_teleop_ee_popup (RobotStateRendererStruc *self)
    {

        GtkWidget *window, *close_button, *vbox_outer;
        BotGtkParamWidget *pw;

        window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
        gtk_window_set_modal(GTK_WINDOW(window), FALSE);
        gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
        gtk_window_stick(GTK_WINDOW(window));
        gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
        int wx,wy;
        wx=500;wy=200;
        gtk_window_set_default_size(GTK_WINDOW(window), wx, wy);
        /*gint pos_x, pos_y;
        gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
        pos_x+=125;    pos_y-=75;
        gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);*/
        
        
        // move dock to account for viewer movement
        gint root_x, root_y;
        gtk_window_get_position (GTK_WINDOW(self->viewer->window), &root_x, &root_y);
        gint width, height;
        gtk_window_get_size(GTK_WINDOW(self->viewer->window),&width,&height);
        gint pos_x, pos_y;
        pos_x=root_x+0.5*width-wx/2;    pos_y=root_y+0.9*height-wy/2;
     
         gint current_pos_x, current_pos_y;
         if((fabs(current_pos_x-pos_x)+fabs(current_pos_y-pos_y))>1)
            gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
        
        
        
        //gtk_widget_set_size_request (window, 300, 250);
        //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
        gtk_window_set_title(GTK_WINDOW(window), "Teleop EE");
        gtk_container_set_border_width(GTK_CONTAINER(window), 5);
        pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
        self->active_ee = drc::ee_cartesian_adjust_t::RIGHT_HAND;
        self->active_res =0.002;
        self->active_angres=1;
        bot_gtk_param_widget_add_enum(pw, PARAM_SELECT_EE_TYPE, BOT_GTK_PARAM_WIDGET_MENU,self->active_ee, 
                                       "Left hand", drc::ee_cartesian_adjust_t::LEFT_HAND, "Right Hand", drc::ee_cartesian_adjust_t::RIGHT_HAND, NULL);
        bot_gtk_param_widget_add_double(pw, PARAM_SELECT_RES, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.001, 0.05, .001, self->active_res);                                
        bot_gtk_param_widget_add_double(pw, PARAM_SELECT_ANG_RES, BOT_GTK_PARAM_WIDGET_SPINBOX, 1, 10, 1, self->active_angres);                                          
        close_button = gtk_button_new_with_label ("Close");
          
       
       GtkWidget  *px_button,*mx_button,*py_button,*my_button,*pz_button,*mz_button,*pr_button,*mr_button,*pp_button,*mp_button,*pyaw_button,*myaw_button;

       px_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_FORWARD);//gtk_button_new_with_label ("+x");
       mx_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_BACK);//gtk_button_new_with_label ("-x");
       py_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_UP);//gtk_button_new_with_label ("+z");
       my_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_DOWN);//gtk_button_new_with_label ("-z");
       pz_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_UP);//gtk_button_new_with_label ("+y");
       mz_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_DOWN);//gtk_button_new_with_label ("-y");
       pr_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_UP);//gtk_button_new_with_label ("+r");
       mr_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_DOWN);//gtk_button_new_with_label ("-r");
       pp_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_UP);//gtk_button_new_with_label ("+r");
       mp_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_DOWN);//gtk_button_new_with_label ("-r");
       pyaw_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_UP);//gtk_button_new_with_label ("+r");
       myaw_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_DOWN);//gtk_button_new_with_label ("-r");     
       
      self->teleop_error_entry = (GtkWidget *) gtk_entry_new();  
      std::stringstream oss;
      double error = 0;
      oss << error;
      string str = oss.str();
      gtk_entry_set_text(GTK_ENTRY(self->teleop_error_entry),str.c_str()); 
      gtk_entry_set_width_chars(GTK_ENTRY(self->teleop_error_entry),(gint)10);
      
   
      
      GtkWidget *vbox;
      GtkWidget * xz_label = gtk_label_new ("XZ");
      vbox = gtk_vbox_new(TRUE, 3);
      gtk_box_pack_start (GTK_BOX (vbox), pz_button, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox), xz_label, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox), mz_button, FALSE, FALSE, 0);
      
      GtkWidget *hbox;
      hbox = gtk_hbox_new(FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox), mx_button, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox),  vbox, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox), px_button, FALSE, FALSE, 0);
      
      GtkWidget *vbox2;
      GtkWidget * null_label2 = gtk_label_new ("Y");
      vbox2 = gtk_vbox_new(TRUE, 3);
      gtk_box_pack_start (GTK_BOX (vbox2), py_button, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox2), null_label2, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox2), my_button, FALSE, FALSE, 0);
      
      GtkWidget *vbox3;
      GtkWidget * null_label3 = gtk_label_new ("R");
      vbox3 = gtk_vbox_new(TRUE, 3);
      gtk_box_pack_start (GTK_BOX (vbox3), pr_button, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox3), null_label3, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox3), mr_button, FALSE, FALSE, 0);
      
      GtkWidget *vbox4;
      GtkWidget * null_label4 = gtk_label_new ("Pit");
      vbox4 = gtk_vbox_new(TRUE, 3);
      gtk_box_pack_start (GTK_BOX (vbox4), pp_button, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox4), null_label4, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox4), mp_button, FALSE, FALSE, 0);
      
      GtkWidget *vbox5;
      GtkWidget * null_label5 = gtk_label_new ("Yaw");
      vbox5 = gtk_vbox_new(TRUE, 3);
      gtk_box_pack_start (GTK_BOX (vbox5), pyaw_button, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox5), null_label5, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (vbox5), myaw_button, FALSE, FALSE, 0);    
      
      
      GtkWidget *hbox_outer;
      GtkWidget * null_label6 = gtk_label_new ("  ");
      hbox_outer = gtk_hbox_new(FALSE,10);
      gtk_box_pack_start (GTK_BOX (hbox_outer), hbox, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox_outer), null_label6, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox_outer), self->teleop_error_entry, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox_outer), vbox2, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox_outer), vbox3, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox_outer), vbox4, FALSE, FALSE, 0);
      gtk_box_pack_start (GTK_BOX (hbox_outer), vbox5, FALSE, FALSE, 0);
            
      self->teleop_popup = window;
      g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_teleop_ee_widget_changed), self);
      g_signal_connect (G_OBJECT (close_button),
                          "clicked",
                          G_CALLBACK (on_popup_close),
                          (gpointer) window);
      g_signal_connect(G_OBJECT(pw), "destroy",
                         G_CALLBACK(on_teleop_ee_popup_close), self); 
      g_signal_connect (G_OBJECT (mx_button),"clicked",G_CALLBACK (on_teleop_button_clicked),self);
      g_signal_connect (G_OBJECT (px_button),"clicked",G_CALLBACK (on_teleop_button_clicked),self);
      g_signal_connect (G_OBJECT (mz_button),"clicked",G_CALLBACK (on_teleop_button_clicked),self);
      g_signal_connect (G_OBJECT (pz_button),"clicked",G_CALLBACK (on_teleop_button_clicked),self);      
      g_signal_connect (G_OBJECT (py_button),"clicked",G_CALLBACK (on_y_teleop_button_clicked),self);                              
      g_signal_connect (G_OBJECT (my_button),"clicked",G_CALLBACK (on_y_teleop_button_clicked),self);    
      g_signal_connect (G_OBJECT (pr_button),"clicked",G_CALLBACK (on_r_teleop_button_clicked),self);                              
      g_signal_connect (G_OBJECT (mr_button),"clicked",G_CALLBACK (on_r_teleop_button_clicked),self); 
      g_signal_connect (G_OBJECT (pp_button),"clicked",G_CALLBACK (on_p_teleop_button_clicked),self);                              
      g_signal_connect (G_OBJECT (mp_button),"clicked",G_CALLBACK (on_p_teleop_button_clicked),self); 
      g_signal_connect (G_OBJECT (pyaw_button),"clicked",G_CALLBACK (on_yaw_teleop_button_clicked),self);                              
      g_signal_connect (G_OBJECT (myaw_button),"clicked",G_CALLBACK (on_yaw_teleop_button_clicked),self);       
      
      vbox_outer = gtk_vbox_new (FALSE, 3);
      gtk_box_pack_start (GTK_BOX (vbox_outer), GTK_WIDGET(pw), FALSE, FALSE, 5);
      gtk_box_pack_start (GTK_BOX (vbox_outer), hbox_outer, FALSE, FALSE, 0);
      gtk_box_pack_end (GTK_BOX (vbox_outer), close_button, FALSE, FALSE, 5);
      gtk_container_add (GTK_CONTAINER (window), vbox_outer);
      gtk_widget_show_all(window); 
    }
    //------------------------------------------------------------------
 } // end namespace 
 
 #endif
