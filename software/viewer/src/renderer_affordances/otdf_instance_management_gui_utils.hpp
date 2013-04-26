#ifndef OTDF_INSTANCE_MANAGEMENT_GUI_UTILS_HPP
#define OTDF_INSTANCE_MANAGEMENT_GUI_UTILS_HPP
#include "renderer_affordances.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
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
  //------------------------------------------------------------------
  // PARAM adjust popup management


  //------------------
  static void on_adjust_params_popup_close (BotGtkParamWidget *pw, void *user)
  {

    RendererAffordances *self = (RendererAffordances*) user;
//    std::string instance_name=  (self->instance_selection_ptr);
//    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
//    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    typedef std::map<std::string, double > params_mapType;
    //for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
    for( params_mapType::const_iterator it2 = self->otdf_instance_hold._otdf_instance->params_map_.begin(); it2!=self->otdf_instance_hold._otdf_instance->params_map_.end(); it2++) 
    { 
      double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
      //std::cout << it->first << ": " << t << std::endl;
      //it->second._otdf_instance->setParam(it2->first, t);
      self->otdf_instance_hold._otdf_instance->setParam(it2->first, t);
    }
    // regen kdl::tree and reset fksolver
    // regen link tfs and shapes for display
    //update_OtdfInstanceStruc(it->second);
    self->otdf_instance_hold._otdf_instance->update();
    self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
    self->selection_hold_on=false;
   
    //was AFFORDANCE_FIT
    //std::string otdf_type = string(self->otdf_instance_hold.otdf_type);
    publish_otdf_instance_to_affstore("AFFORDANCE_TRACK",(self->otdf_instance_hold.otdf_type),self->otdf_instance_hold.uid,self->otdf_instance_hold._otdf_instance,self); 
    bot_viewer_request_redraw(self->viewer);
  }  
  //------------------
  static void on_otdf_adjust_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {

    RendererAffordances *self = (RendererAffordances*) user;
//    std::string instance_name=  (self->instance_selection_ptr);
//    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
//    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    typedef std::map<std::string, double > params_mapType;
    //for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
    for( params_mapType::const_iterator it2 = self->otdf_instance_hold._otdf_instance->params_map_.begin(); it2!=self->otdf_instance_hold._otdf_instance->params_map_.end(); it2++)
    { 
      if(!strcmp(name, it2->first.c_str())) {
          double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
      //std::cout << it->first << ": " << t << std::endl;
      //it->second._otdf_instance->setParam(it2->first, t);
      self->otdf_instance_hold._otdf_instance->setParam(it2->first, t);
      }
    }
    
    self->otdf_instance_hold._otdf_instance->update(); //update_OtdfInstanceStruc(it->second);
    self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
    bot_viewer_request_redraw(self->viewer);// gives realtime feedback of the geometry changing.
  }
  

  
  
//------------------------------------------------------------------
// DOF adjust popup management
  static void on_adjust_dofs_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    std::string instance_name=  self->instance_selection;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    it->second._gl_object->set_future_state_changing(false);
    
  //   TODO: Send publish desired affordance state command msg    
  
  }

  static void on_otdf_adjust_dofs_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    std::string instance_name=  self->instance_selection;
    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    if(!it->second._gl_object->is_future_state_changing()) {
    
      it->second._gl_object->set_future_state_changing(true);

     // clear previously accumulated motion states for all dependent bodies
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
      while (hand_it!=self->sticky_hands.end()) 
      {
         std::string hand_name = std::string(hand_it->second.object_name);
         if (hand_name == (instance_name))
         {
            hand_it->second._gl_hand->clear_desired_body_motion_history();
         }
         hand_it++;
      }
     }//end if(!it->second._gl_object->is_future_state_changing())

    // get desired state from popup sliders
    KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
    std::map<std::string, double> jointpos_in;
      
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double desired_dof_pos = 0;
      if(joint->second->type!=(int) otdf::Joint::FIXED) {
          desired_dof_pos =  bot_gtk_param_widget_get_double (pw, joint->first.c_str());
          jointpos_in.insert(make_pair(joint->first, desired_dof_pos)); 
       cout <<  joint->first << " dof changed to " << desired_dof_pos << endl;
      }
     }
    it->second._gl_object->set_future_state(T_world_object,jointpos_in); 

  
    bot_viewer_request_redraw(self->viewer);
  }
//------------------------------------------------------------------
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
    gint pos_x, pos_y;
    gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
    pos_x+=125;    pos_y-=75;
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "Adjust Params");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    std::string instance_name=  self->instance_selection;
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
    

    // create a temp copy of the selected otdf instance to make modifications to.    
    if(!self->selection_hold_on) { // Assuming only one object instance is changed at any given time
      self->otdf_instance_hold.uid=it->second.uid;
     
      self->otdf_instance_hold.otdf_type = it->second.otdf_type;
      //self->otdf_instance_hold.otdf_type = new string((*it->second.otdf_type));    // SEGFAULTS With Strings
      self->otdf_instance_hold._otdf_instance = otdf::duplicateOTDFInstance(it->second._otdf_instance);
      self->otdf_instance_hold._gl_object.reset();
      self->otdf_instance_hold._collision_detector.reset();
      self->otdf_instance_hold._collision_detector = shared_ptr<Collision_Detector>(new Collision_Detector());     
      self->otdf_instance_hold._gl_object = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(self->otdf_instance_hold._otdf_instance,self->otdf_instance_hold._collision_detector,true,"otdf_instance_hold"));
      self->otdf_instance_hold._otdf_instance->update();
      self->otdf_instance_hold._gl_object->set_state(self->otdf_instance_hold._otdf_instance);
      self->selection_hold_on=true;
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
    gint pos_x, pos_y;
    gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
    pos_x+=125;    pos_y-=75;
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
    //gtk_widget_set_size_request (window, 300, 250);
    //gtk_window_set_resizable(GTK_WINDOW(window), FALSE);
    gtk_window_set_title(GTK_WINDOW(window), "Adjust Dofs");
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    std::string instance_name=  self->instance_selection;

    typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    // Need tarcked joint positions of all objects.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double current_dof_position = 0;// TODO: dof pos tracking
      if(joint->second->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
        if(joint->second->type==(int) otdf::Joint::CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI, 2*M_PI, .01, current_dof_position); 
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
        if(jp_it->second->joint_set[i]->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
          if(jp_it->second->joint_set[i]->type==(int) otdf::Joint::CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          -2*M_PI, 2*M_PI, .01, current_dof_position); 
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


  // ================================================================================
  // First Stage Popup of OTDF instance management

  static void on_otdf_instance_management_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;

    // int selection = bot_gtk_param_widget_get_enum (pw, PARAM_OTDF_INSTANCE_SELECT);
    const char *instance_name;
    instance_name = bot_gtk_param_widget_get_enum_str( pw, PARAM_OTDF_INSTANCE_SELECT );
    self->instance_selection  = std::string(instance_name);

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
      
      // send message to affordance store
      string name = instance_name;
      size_t pos = name.find("_"); // name of format otdftype_uid
      if(pos == string::npos){
        cout << "Error parsing instance_name: " << instance_name << endl;
      }else{ 
        // create and send delete affordance message
        drc::affordance_plus_t aff;
        aff.aff.aff_store_control = drc::affordance_t::DELETE;
        aff.aff.otdf_type = name.substr(0,pos);
        aff.aff.uid = atoi(name.substr(pos+1).c_str());
        aff.aff.map_id = 0;
        aff.aff.nparams = 0;
        aff.aff.nstates = 0;
        aff.npoints = 0;
        aff.ntriangles = 0;
        self->lcm->publish("AFFORDANCE_FIT", &aff);
        cout << "Delete message sent for: " << name << endl;
      }

     if(it!=self->instantiated_objects.end())
     {
        self->instantiated_objects.erase(it);
        if(self->object_selection==std::string(instance_name))
        {
          self->link_selection = " ";
          self->object_selection = " ";
        }  

        typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
       
        sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
        while (hand_it!=self->sticky_hands.end()) {
            std::string hand_name = std::string(hand_it->second.object_name);
           if (hand_name == std::string(instance_name))
           {
              if(self->stickyhand_selection==hand_it->first)
                 self->stickyhand_selection = " ";
              self->sticky_hands.erase(hand_it++);
           }
           else
              hand_it++;
        } 

        bot_viewer_request_redraw(self->viewer);
      }
    }
    else if(!strcmp(name,PARAM_OTDF_INSTANCE_CLEAR_ALL)) {
      fprintf(stderr,"\nClearing Instantiated Objects\n");
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      self->instantiated_objects.clear();
      self->sticky_hands.clear();
      for( std::map<std::string,int >::iterator it = self->instance_cnt.begin(); it!=self->instance_cnt.end(); it++)
      { 
       it->second = 0;
      }
     self->link_selection = " ";
     self->object_selection = " ";
     self->stickyhand_selection = " ";
      bot_viewer_request_redraw(self->viewer);
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
    gint pos_x, pos_y;
    gtk_window_get_position(GTK_WINDOW(window),&pos_x,&pos_y);
    pos_x+=125;    pos_y-=75;
    gtk_window_move(GTK_WINDOW(window),pos_x,pos_y);
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
