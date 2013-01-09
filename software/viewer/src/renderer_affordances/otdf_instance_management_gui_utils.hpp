#ifndef OTDF_INSTANCE_MANAGEMENT_GUI_UTILS_HPP
#define OTDF_INSTANCE_MANAGEMENT_GUI_UTILS_HPP
#include "renderer_affordances.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision_detection;
using namespace renderer_affordances;



namespace renderer_affordances_gui_utils
{


  static gboolean on_popup_close (GtkButton* button, GtkWidget* pWindow)
  {
    gtk_widget_destroy (pWindow);
    return TRUE;
  }
  
  // ================================================================================
  // Second Stage Popup of OTDF instance management for Adjust Params and Dofs using Spinboxes and Sliders
  
  static void on_adjust_params_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    std::string instance_name=  (*self->instance_selection_ptr);
    //typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    typedef std::map<std::string, double > params_mapType;
    // for( params_mapType::const_iterator it2 = it->second->params_map_.begin(); it2!=it->second->params_map_.end(); it2++)
    for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
    { 
      double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
      //std::cout << it->first << ": " << t << std::endl;
      //it->second->setParam(it->first, t);
      it->second._otdf_instance->setParam(it2->first, t);
    }
    // it->second->update();

    // regen kdl::tree and reset fksolver
    // regen link tfs and shapes for display
    update_OtdfInstanceStruc(it->second);
    bot_viewer_request_redraw(self->viewer);
  }

  static void on_otdf_adjust_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;

    std::string instance_name=  (*self->instance_selection_ptr);
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    typedef std::map<std::string, double > params_mapType;
    for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
    { 
      if(!strcmp(name, it2->first.c_str())) {
          double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
        //std::cout << it2->first << ": " << t << std::endl;
        it->second._otdf_instance->setParam(it2->first, t);
      }
    }
    // gives realtime feedback of the geometry changing.
    update_OtdfInstanceStruc(it->second);
    bot_viewer_request_redraw(self->viewer);
  }

  static void on_adjust_dofs_popup_close (BotGtkParamWidget *pw, void *user)
  {
  //     RendererAffordances *self = (RendererAffordances*) user;
  //   std::string instance_name=  (*self->instance_selection_ptr);
  //   typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
  //   object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
  // TODO: Send publish affordance command msg
  }

  static void on_otdf_adjust_dofs_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
  //   RendererAffordances *self = (RendererAffordances*) user;
  //   std::string instance_name=  (*self->instance_selection_ptr);
  //   typedef std::map<std::string, boost::shared_ptr<otdf::ModelInterface> > object_instance_map_type_;
  //   object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
  //  
  //TODO: do something
  }

  static void spawn_adjust_params_popup (RendererAffordances *self)
  {

    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw;

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "Adjust Params");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    std::string instance_name=  (*self->instance_selection_ptr);

    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    for(std::vector<std::string>::const_iterator it2 = it->second._otdf_instance->params_order_.begin(); it2 != it->second._otdf_instance->params_order_.end(); ++it2) 
    {
      double inc = it->second._otdf_instance->param_properties_map_[*it2].inc;
      double min = it->second._otdf_instance->param_properties_map_[*it2].min_value;
      double max = it->second._otdf_instance->param_properties_map_[*it2].max_value;
      double value = it->second._otdf_instance->params_map_[*it2];
      bot_gtk_param_widget_add_double(pw, (*it2).c_str(), BOT_GTK_PARAM_WIDGET_SPINBOX,
       min, max, inc, value); 
    }

    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_adjust_param_widget_changed), self);


    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),
                  "clicked",
                  G_CALLBACK (on_popup_close),
                  (gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy",
    G_CALLBACK(on_adjust_params_popup_close), self); 


    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 
  }


  static void spawn_adjust_dofs_popup (RendererAffordances *self)
  {

    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw;

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "Adjust Dofs");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    std::string instance_name=  (*self->instance_selection_ptr);

    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    enum
    {
      UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    };

    // Need tarcked joint positions of all objects.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double current_dof_position = 0;// TODO: dof pos tracking
      if(joint->second->type!=(int) FIXED) { // All joints that not of the type FIXED.
        if(joint->second->type==(int) CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, 0, M_PI, .01, current_dof_position); 
        }
        else{
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          joint->second->limits->lower, joint->second->limits->upper, .01, current_dof_position);
        }   
      }
    }
    //Have to handle joint_patterns separately   
    // DoF of all joints in joint patterns.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
    for (jp_mapType::iterator jp_it = it->second._otdf_instance->joint_patterns_.begin();jp_it != it->second._otdf_instance->joint_patterns_.end(); jp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
      {
        double current_dof_position = 0;// TODO: dof pos tracking
        if(jp_it->second->joint_set[i]->type!=(int) FIXED) { // All joints that not of the type FIXED.
          if(jp_it->second->joint_set[i]->type==(int) CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          0, M_PI, .01, current_dof_position); 
          }
          else{
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          jp_it->second->joint_set[i]->limits->lower, jp_it->second->joint_set[i]->limits->upper, .01, current_dof_position);
          }   
        } // end if         
      } // end for all joints in jp
    }// for all joint patterns

    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_adjust_dofs_widget_changed), self);


    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),
                    "clicked",
                    G_CALLBACK (on_popup_close),
                    (gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy",
      G_CALLBACK(on_adjust_dofs_popup_close), self); 


    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);
    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window); 
  }



  static void publish_eegoal(boost::shared_ptr<lcm::LCM> &_lcm, OtdfInstanceStruc &instance_struc, std::string channel, KDL::Frame &T_body_world)
  {
    drc::ee_goal_t goalmsg;

    double x,y,z,w;

    // desired ee position in world frame
    KDL::Frame T_world_ee,T_body_ee;
    T_world_ee.p[0]= instance_struc._otdf_instance->getParam("x");
    T_world_ee.p[1]= instance_struc._otdf_instance->getParam("y");
    T_world_ee.p[2]= instance_struc._otdf_instance->getParam("z");
    T_world_ee.M =  KDL::Rotation::RPY(instance_struc._otdf_instance->getParam("roll"),
                                       instance_struc._otdf_instance->getParam("pitch"),
                                       instance_struc._otdf_instance->getParam("yaw"));
          
    //T_body_world = self->robotStateListener->T_body_world; //KDL::Frame::Identity(); // must also have robot state listener.

    // desired ee position wrt to robot body.
    T_body_ee = T_body_world*T_world_ee;

    T_body_ee.M.GetQuaternion(x,y,z,w);

    goalmsg.ee_goal_pos.translation.x = T_body_ee.p[0];
    goalmsg.ee_goal_pos.translation.y = T_body_ee.p[1];
    goalmsg.ee_goal_pos.translation.z = T_body_ee.p[2];

    goalmsg.ee_goal_pos.rotation.x = x;
    goalmsg.ee_goal_pos.rotation.y = y;
    goalmsg.ee_goal_pos.rotation.z = z;
    goalmsg.ee_goal_pos.rotation.w = w;

    goalmsg.ee_goal_twist.linear_velocity.x = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.y = 0.0;
    goalmsg.ee_goal_twist.linear_velocity.z = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.x = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.y = 0.0;
    goalmsg.ee_goal_twist.angular_velocity.z = 0.0;

    goalmsg.num_chain_joints  = 6;
    // No specified posture bias
    goalmsg.use_posture_bias  = false;
    goalmsg.joint_posture_bias.resize(goalmsg.num_chain_joints);
    goalmsg.chain_joint_names.resize(goalmsg.num_chain_joints);
    for(int i = 0; i < goalmsg.num_chain_joints; i++){
    goalmsg.joint_posture_bias[i]=0;
    goalmsg.chain_joint_names[i]= "dummy_joint_names";
    }

    // Publish the message
    goalmsg.halt_ee_controller = false;

    _lcm->publish(channel, &goalmsg);
  }
  
  // ================================================================================
  // First Stage Popup of OTDF instance management

  static void on_otdf_instance_management_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;

    // int selection = bot_gtk_param_widget_get_enum (pw, PARAM_OTDF_INSTANCE_SELECT);
    const char *instance_name;
    instance_name = bot_gtk_param_widget_get_enum_str( pw, PARAM_OTDF_INSTANCE_SELECT );
    (*self->instance_selection_ptr)  = std::string(instance_name);

    if(!strcmp(name,PARAM_OTDF_ADJUST_PARAM)) {
      spawn_adjust_params_popup(self);
    }
    else if(!strcmp(name,PARAM_OTDF_ADJUST_DOF)) {
      spawn_adjust_dofs_popup(self);
    }
    else if(!strcmp(name,PARAM_OTDF_INSTANCE_CLEAR)) {
      fprintf(stderr,"\nClearing Selected Instance\n");
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->instantiated_objects.find(std::string(instance_name));
      self->instantiated_objects.erase(it);
      bot_viewer_request_redraw(self->viewer);
    }
    else if(!strcmp(name,PARAM_OTDF_INSTANCE_CLEAR_ALL)) {
      fprintf(stderr,"\nClearing Instantiated Objects\n");
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      self->instantiated_objects.clear();
      for( std::map<std::string,int >::iterator it = self->instance_cnt.begin(); it!=self->instance_cnt.end(); it++)
      { 
       it->second = 0;
      }
      bot_viewer_request_redraw(self->viewer);
    }
    else if(!strcmp(name,PARAM_OTDF_REACH_OBJECT_L)) {
      fprintf(stderr,"\nReaching centroid of selected Object\n");
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->instantiated_objects.find(std::string(instance_name));
      KDL::Frame T_body_world = self->robotStateListener->T_body_world;
      publish_eegoal( self->lcm, it->second, "LWRISTROLL_LINK_GOAL",T_body_world);
    }
    else if(!strcmp(name,PARAM_OTDF_REACH_OBJECT_R)) {
      fprintf(stderr,"\nReaching centroid of selected Object\n");
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->instantiated_objects.find(std::string(instance_name));
      KDL::Frame T_body_world = self->robotStateListener->T_body_world;
      publish_eegoal( self->lcm, it->second, "RWRISTROLL_LINK_GOAL",T_body_world);    
    }
  }

  static void spawn_instance_management_popup (RendererAffordances *self)
  {
    GtkWidget *window, *close_button, *vbox;
    BotGtkParamWidget *pw; 

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(self->viewer->window));
    gtk_window_set_modal(GTK_WINDOW(window), FALSE);
    gtk_window_set_decorated  (GTK_WINDOW(window),FALSE);
    gtk_window_stick(GTK_WINDOW(window));
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_MOUSE);
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 250);
    gtk_window_set_title(GTK_WINDOW(window), "Instance Management");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    int num_otdf_instances;
    char ** otdf_instance_names;
    int * otdf_instance_nums;

    if( self->instantiated_objects.size() > 0)
    {
      num_otdf_instances = self->instantiated_objects.size();
      otdf_instance_names =(char **) calloc(num_otdf_instances, sizeof(char *));
      otdf_instance_nums = (int *)calloc(num_otdf_instances, sizeof(int));

      unsigned int i = 0;  
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      for( object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
      { 
        std::string instance_name = it->first;
        otdf_instance_names[i] = (char *) instance_name.c_str();
        otdf_instance_nums[i] =i;
        // std::cout << "Instance:  " << instance_name  << " i: "<< i << std::endl;
        // self->instantiated_objects_id.insert(std::make_pair(i,instance_name));
        ++i;
      }
    }
    else 
    {
      num_otdf_instances = 1;
      otdf_instance_names =(char **) calloc(num_otdf_instances, sizeof(char *));
      otdf_instance_nums = (int *)calloc(num_otdf_instances, sizeof(int));

      std::string instance_name = "No objects Instantiated";
      otdf_instance_names[0]= (char *) instance_name.c_str();
      otdf_instance_nums[0] =0; 
    }

    bot_gtk_param_widget_add_enumv (pw, PARAM_OTDF_INSTANCE_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
                                  0,
                                  num_otdf_instances,
                                  (const char **)  otdf_instance_names,
                                  otdf_instance_nums);

    if( self->instantiated_objects.size() > 0)
    {
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_PARAM, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_ADJUST_DOF, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_INSTANCE_CLEAR, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_INSTANCE_CLEAR_ALL, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_REACH_OBJECT_L, NULL);
      bot_gtk_param_widget_add_buttons(pw,PARAM_OTDF_REACH_OBJECT_R, NULL);
    }

    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_instance_management_widget_changed), self);

    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),
                  "clicked",
                  G_CALLBACK (on_popup_close),
                  (gpointer) window);

    vbox = gtk_vbox_new (FALSE, 3);
    gtk_box_pack_end (GTK_BOX (vbox), close_button, FALSE, FALSE, 5);

    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET(pw), FALSE, FALSE, 5);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show_all(window);
  }

}
#endif
