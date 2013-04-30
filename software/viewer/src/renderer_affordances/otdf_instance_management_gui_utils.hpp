#ifndef OTDF_INSTANCE_MANAGEMENT_GUI_UTILS_HPP
#define OTDF_INSTANCE_MANAGEMENT_GUI_UTILS_HPP
#include "renderer_affordances.hpp"
#include "lcm_utils.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
using namespace renderer_affordances;
using namespace renderer_affordances_lcm_utils;

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
//    string instance_name=  (self->instance_selection_ptr);
//    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
//    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    typedef map<string, double > params_mapType;
    //for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
    for( params_mapType::const_iterator it2 = self->otdf_instance_hold._otdf_instance->params_map_.begin(); it2!=self->otdf_instance_hold._otdf_instance->params_map_.end(); it2++) 
    { 
      double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
      //cout << it->first << ": " << t << endl;
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
    //string otdf_type = string(self->otdf_instance_hold.otdf_type);
    publish_otdf_instance_to_affstore("AFFORDANCE_TRACK",(self->otdf_instance_hold.otdf_type),self->otdf_instance_hold.uid,self->otdf_instance_hold._otdf_instance,self); 
    bot_viewer_request_redraw(self->viewer);
  }  
  //------------------
  static void on_otdf_adjust_param_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {

    RendererAffordances *self = (RendererAffordances*) user;
//    string instance_name=  (self->instance_selection_ptr);
//    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
//    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    typedef map<string, double > params_mapType;
    //for( params_mapType::const_iterator it2 = it->second._otdf_instance->params_map_.begin(); it2!=it->second._otdf_instance->params_map_.end(); it2++)
    for( params_mapType::const_iterator it2 = self->otdf_instance_hold._otdf_instance->params_map_.begin(); it2!=self->otdf_instance_hold._otdf_instance->params_map_.end(); it2++)
    { 
      if(!strcmp(name, it2->first.c_str())) {
          double t = bot_gtk_param_widget_get_double (pw,it2->first.c_str());
      //cout << it->first << ": " << t << endl;
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
    string instance_name=  self->instance_selection;
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    it->second._gl_object->set_future_state_changing(false);
  //   TODO: Send publish desired affordance state command msg    
  
  }
//------------------------------------------------------------------ 
  static void on_otdf_adjust_dofs_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    string instance_name=  self->instance_selection;
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    if(!it->second._gl_object->is_future_state_changing()) 
    {
        it->second._gl_object->set_future_state_changing(true);

       // clear previously accumulated motion states for all dependent bodies
       typedef map<string, StickyHandStruc > sticky_hands_map_type_;
        sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
        while (hand_it!=self->sticky_hands.end()) 
        {
           string hand_name = string(hand_it->second.object_name);
           if (hand_name == (instance_name))
           {
              hand_it->second._gl_hand->clear_desired_body_motion_history();
           }
           hand_it++;
        }

        typedef map<string, StickyFootStruc > sticky_feet_map_type_;
        sticky_feet_map_type_::iterator foot_it = self->sticky_feet.begin();
        while (foot_it!=self->sticky_feet.end()) 
        {
           string foot_name = string(foot_it->second.object_name);
           if (foot_name == (instance_name))
           {
              foot_it->second._gl_foot->clear_desired_body_motion_history();
           }
           foot_it++;
        }
      
     }//end if(!it->second._gl_object->is_future_state_changing())

    // get desired state from popup sliders
    KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
    map<string, double> jointpos_in;
      
    typedef map<string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double desired_dof_pos = 0;
      if(joint->second->type!=(int) otdf::Joint::FIXED) {
          desired_dof_pos =  bot_gtk_param_widget_get_double (pw, joint->first.c_str())*(M_PI/180);
          jointpos_in.insert(make_pair(joint->first, desired_dof_pos)); 
       cout <<  joint->first << " dof changed to " << desired_dof_pos*(180/M_PI) << endl;
      }
     }
     
    if(!it->second._gl_object->is_future_state_changing())   {
       it->second._gl_object->set_future_state_changing(true);
    }  
    it->second._gl_object->set_future_state(T_world_object,jointpos_in); 

    //cout <<"    self->motion_trail_log_enabled "<< self->motion_trail_log_enabled << endl;
  
    bot_viewer_request_redraw(self->viewer);
  }
  
  
//------------------------------------------------------------------  
  static void on_otdf_dof_range_widget_popup_close (BotGtkParamWidget *pw, void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    string instance_name=  self->instance_selection;
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    self->popup_widget_name_list.clear();
    it->second._gl_object->set_future_state_changing(false);
    it->second._gl_object->disable_future_display();
    // reset object to what it was at the time of popup
    it->second._gl_object->set_state(self->otdf_T_world_body_hold,self->otdf_current_jointpos_hold);
    it->second._gl_object->set_future_state(self->otdf_T_world_body_hold,self->otdf_current_jointpos_hold);
    cout << "dof_range_widget popup close\n";

    map<string, vector<KDL::Frame> > ee_frames_map;
    map<string, vector<drc::affordance_index_t> > ee_frame_affindices_map;

      // Publish EE Range goals for associated sticky hands 
      typedef map<string, StickyHandStruc > sticky_hands_map_type_;
      for(sticky_hands_map_type_::const_iterator hand_it = self->sticky_hands.begin(); hand_it!=self->sticky_hands.end(); hand_it++)
      {
         string host_name = hand_it->second.object_name;
         if (host_name == (it->first))
         {
         
            string ee_name;
            if(hand_it->second.hand_type==0)
              ee_name ="left_palm";    
            else if(hand_it->second.hand_type==1)   
               ee_name ="right_palm";    
            else
               cout << "unknown hand_type in on_otdf_dof_range_widget_popup_close\n";   
               
            // if ee_name already exists in ee_frames_map, redundant ee_frames
            // e.g two right sticky hands on the same object.
            map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
            if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
              return;   
            }
          
            KDL::Frame  T_geometry_ee = KDL::Frame::Identity(); 
            if(!hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_ee))
                cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
         
         
            vector<KDL::Frame> T_world_geometry_frames;        
            string seed_geometry_name = hand_it->second.geometry_name;
            self->dofRangeFkQueryHandler->getLinkFrames(seed_geometry_name,T_world_geometry_frames);
            std::string dof_name;
            vector<double> dof_values;  
            if(!self->dofRangeFkQueryHandler->getAssociatedDoFNameAndVal(seed_geometry_name,dof_name,dof_values))
            {
              cerr << "ERROR in getAssociatedDoFNameAndVal \n";
              return;
            }
            
            int num_of_incs = T_world_geometry_frames.size();
            vector<KDL::Frame> T_world_ee_frames;
            vector<drc::affordance_index_t> frame_affindices;
            for(size_t i=0;i<(size_t)num_of_incs;i++){
              KDL::Frame  T_world_ee = T_world_geometry_frames[i]*T_geometry_ee;     
              T_world_ee_frames.push_back(T_world_ee);
              drc::affordance_index_t aff_index;
              aff_index.utime=(int64_t)i;
              aff_index.aff_type = it->second.otdf_type; 
              aff_index.aff_uid = it->second.uid;   
              aff_index.num_ees =1;  
              aff_index.ee_name.push_back(ee_name); 
              aff_index.dof_name.push_back(dof_name);     
              aff_index.dof_value.push_back(dof_values[i]); 
        
              frame_affindices.push_back(aff_index);
            } 
           ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
           ee_frame_affindices_map.insert(make_pair(ee_name, frame_affindices));   
         } // end if (host_name == (it->first))
      }
        
       // Publish EE Range goals for associated stick feet 
     typedef map<string, StickyFootStruc > sticky_feet_map_type_;
      for(sticky_feet_map_type_::const_iterator foot_it = self->sticky_feet.begin(); foot_it!=self->sticky_feet.end(); foot_it++)
      {
         string host_name = foot_it->second.object_name;
         if (host_name == (it->first))
         {
         
            string ee_name;
            if(foot_it->second.foot_type==0)
               ee_name ="l_foot";    
            else if(foot_it->second.foot_type==1)   
               ee_name ="r_foot";    
            else
               cout << "unknown foot_type in on_otdf_dof_range_widget_popup_close\n";  
               
            // if ee_name already exists in ee_frames_map, redundant ee_frames
            // e.g two right sticky hands on the same object.
            map<string, vector<KDL::Frame> >::const_iterator ee_it = ee_frames_map.find(ee_name);
            if(ee_it!=ee_frames_map.end()){
              cerr<<" ERROR: Cannot of two seeds of the same ee. Please consider deleting redundant seeds\n";
              return;   
            }      
          
            KDL::Frame  T_geometry_ee = KDL::Frame::Identity(); 
            if(!foot_it->second._gl_foot->get_link_frame(ee_name,T_geometry_ee))
                cout <<"ERROR: ee link "<< ee_name << " not found in sticky hand urdf"<< endl;
         
         
            vector<KDL::Frame> T_world_geometry_frames;
            string seed_geometry_name = foot_it->second.geometry_name;
            self->dofRangeFkQueryHandler->getLinkFrames(seed_geometry_name,T_world_geometry_frames);
            std::string dof_name;
            vector<double> dof_values;  
            if(!self->dofRangeFkQueryHandler->getAssociatedDoFNameAndVal(seed_geometry_name,dof_name,dof_values))
            {
              cerr << "ERROR in getAssociatedDoFNameAndVal \n";
              return;
            }

            int num_of_incs = T_world_geometry_frames.size();
            vector<KDL::Frame> T_world_ee_frames;
            vector<drc::affordance_index_t> frame_affindices;
            for(size_t i=0;i<(size_t)num_of_incs;i++){
              KDL::Frame  T_world_ee = T_world_geometry_frames[i]*T_geometry_ee;     
              T_world_ee_frames.push_back(T_world_ee);
              
              drc::affordance_index_t aff_index;
              aff_index.utime=(int64_t)i;
              aff_index.aff_type = it->second.otdf_type; 
              aff_index.aff_uid = it->second.uid; 
              aff_index.num_ees =1;  
              aff_index.ee_name.push_back(ee_name); 
              aff_index.dof_name.push_back(dof_name);     
              aff_index.dof_value.push_back(dof_values[i]); 
              frame_affindices.push_back(aff_index);
            } 
            
           ee_frames_map.insert(make_pair(ee_name, T_world_ee_frames));
           ee_frame_affindices_map.insert(make_pair(ee_name, frame_affindices));    
         }
      
      } 
      
   string channel  ="DESIRED_MANIP_MAP_EE_LOCI"; 
   publish_aff_indexed_traj_opt_constraint(channel, ee_frames_map, ee_frame_affindices_map, self);
    
  }
//------------------------------------------------------------------    
  static void on_otdf_dof_range_widget_changed(BotGtkParamWidget *pw, const char *name,void *user)
  {
    RendererAffordances *self = (RendererAffordances*) user;
    string instance_name=  self->instance_selection;
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);
    
    // get desired dof ranges from popup sliders

    vector<string> dof_names;
    vector<double> dof_min,dof_max;
    map<string, double> current_jointpos_in;
    map<string, double> future_jointpos_in; 
    
    bool doFK= false;
    
    typedef map<string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      
      if(joint->second->type!=(int) otdf::Joint::FIXED) 
      {
      
        double current_dof_vel = 0;
        double current_dof_pos = 0;
        it->second._otdf_instance->getJointState(joint->first,current_dof_pos,current_dof_vel);
        string temp1 = joint->first + "_MIN"; 
        string temp2 = joint->first +"_MAX";   
        vector<string>::const_iterator found1,found2;
        found1 = find (self->popup_widget_name_list.begin(),self->popup_widget_name_list.end(), temp1);
        found2= find (self->popup_widget_name_list.begin(),self->popup_widget_name_list.end(), temp2);
        if ((found1 != self->popup_widget_name_list.end())&&(found2 != self->popup_widget_name_list.end())) 
        {
          unsigned int index1 = found1 - self->popup_widget_name_list.begin();      
          unsigned int index2 = found2 - self->popup_widget_name_list.begin(); 
          double desired_dof_pos_min = 0;
          double desired_dof_pos_max = 0;
          desired_dof_pos_min =  bot_gtk_param_widget_get_double (pw, self->popup_widget_name_list[index1].c_str())*(M_PI/180);
          desired_dof_pos_max =  bot_gtk_param_widget_get_double (pw, self->popup_widget_name_list[index2].c_str())*(M_PI/180);
          
          current_jointpos_in.insert(make_pair(joint->first, desired_dof_pos_min)); 
          future_jointpos_in.insert(make_pair(joint->first, desired_dof_pos_max)); 
          if((current_dof_pos>(desired_dof_pos_min+1e-2))||(current_dof_pos<(desired_dof_pos_max-1e-2)))
          {
            cout <<  joint->first << ":: desired dof range set to " << desired_dof_pos_min*(180/M_PI) << " : "<< desired_dof_pos_max*(180/M_PI)<< endl;
            dof_names.push_back(joint->first);
            dof_min.push_back(desired_dof_pos_min);
            dof_max.push_back(desired_dof_pos_max);
            doFK = true;
          }
        }  
       
      }// end if
     }// end for

     
   if(doFK)
   {
      //doBatchFK given DOF Desired Ranges
      int num_of_increments = 5; // determines no of intermediate holds between dof_min and dof_max;
      self->dofRangeFkQueryHandler->doBatchFK(dof_names,dof_min,dof_max,num_of_increments);
   } 

      // Visualizing how seeds change with dof range changes
      //--------------------------------------
      if(!it->second._gl_object->is_future_state_changing()) {
        it->second._gl_object->set_future_state_changing(true);
      }      
      KDL::Frame T_world_object = it->second._gl_object->_T_world_body;
      it->second._gl_object->set_state(T_world_object,current_jointpos_in);     
      it->second._gl_object->set_future_state(T_world_object,future_jointpos_in);
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

    string instance_name=  self->instance_selection;
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    for(vector<string>::const_iterator it2 = it->second._otdf_instance->params_order_.begin(); it2 != it->second._otdf_instance->params_order_.end(); ++it2) 
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
//------------------------------------------------------------------

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

    string instance_name=  self->instance_selection;

    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    // Need tarcked joint positions of all objects.
    typedef map<string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
      double current_dof_position = 0;// TODO: dof pos tracking
      if(joint->second->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
        if(joint->second->type==(int) otdf::Joint::CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
        }
        else{
          bot_gtk_param_widget_add_double(pw, joint->first.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          joint->second->limits->lower*(180/M_PI), joint->second->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
        }   
      }
    }
    //Have to handle joint_patterns separately   
    // DoF of all joints in joint patterns.
    typedef map<string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
    for (jp_mapType::iterator jp_it = it->second._otdf_instance->joint_patterns_.begin();jp_it != it->second._otdf_instance->joint_patterns_.end(); jp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
      {
        double current_dof_position = 0;// TODO: dof pos tracking
        if(jp_it->second->joint_set[i]->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
          if(jp_it->second->joint_set[i]->type==(int) otdf::Joint::CONTINUOUS) {
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
          }
          else{
          bot_gtk_param_widget_add_double(pw, jp_it->second->joint_set[i]->name.c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
          jp_it->second->joint_set[i]->limits->lower*(180/M_PI), jp_it->second->joint_set[i]->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
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
//------------------------------------------------------------------

  static void spawn_set_dof_range_popup (RendererAffordances *self)
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

    string instance_name=  self->instance_selection;
    //self->popup_widget_name_list.clear(); 

    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator it = self->instantiated_objects.find(instance_name);

    // Need tarcked joint positions of all objects.
    typedef map<string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator joint = it->second._otdf_instance->joints_.begin();joint != it->second._otdf_instance->joints_.end(); joint++)
    {     
        double current_dof_velocity = 0;
        double current_dof_position = 0;
         it->second._otdf_instance->getJointState(joint->first,current_dof_position,current_dof_velocity);
      if(joint->second->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
        if(joint->second->type==(int) otdf::Joint::CONTINUOUS) {
            self->popup_widget_name_list.push_back(joint->first+"_MIN"); 
            bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
            self->popup_widget_name_list.push_back(joint->first+"_MAX");   
            bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
            //bot_gtk_param_widget_add_separator (pw," ");
        }
        else
        {
          self->popup_widget_name_list.push_back(joint->first+"_MIN"); 
          bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,joint->second->limits->lower*(180/M_PI), joint->second->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
          self->popup_widget_name_list.push_back(joint->first+"_MAX"); 
          bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,joint->second->limits->lower*(180/M_PI), joint->second->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
         // bot_gtk_param_widget_add_separator (pw," ");
        }   
      }
    }
    //Have to handle joint_patterns separately   
    // DoF of all joints in joint patterns.
    typedef map<string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
    for (jp_mapType::iterator jp_it = it->second._otdf_instance->joint_patterns_.begin();jp_it != it->second._otdf_instance->joint_patterns_.end(); jp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
      {
        double current_dof_velocity = 0;
        double current_dof_position = 0;
         it->second._otdf_instance->getJointState(jp_it->second->joint_set[i]->name,current_dof_position,current_dof_velocity);
        if(jp_it->second->joint_set[i]->type!=(int) otdf::Joint::FIXED) { // All joints that not of the type FIXED.
          if(jp_it->second->joint_set[i]->type==(int) otdf::Joint::CONTINUOUS) 
          {          
            self->popup_widget_name_list.push_back(jp_it->second->joint_set[i]->name+"_MIN");
            bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
            self->popup_widget_name_list.push_back(jp_it->second->joint_set[i]->name+"_MAX");  
            bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER, -2*M_PI*(180/M_PI), 2*M_PI*(180/M_PI), .01, current_dof_position*(180/M_PI)); 
            //bot_gtk_param_widget_add_separator (pw," ");
          }
          else
          {
            self->popup_widget_name_list.push_back(jp_it->second->joint_set[i]->name+"_MIN");  
            bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
            jp_it->second->joint_set[i]->limits->lower*(180/M_PI), jp_it->second->joint_set[i]->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
            self->popup_widget_name_list.push_back(jp_it->second->joint_set[i]->name+"_MAX"); 
            bot_gtk_param_widget_add_double(pw, self->popup_widget_name_list[self->popup_widget_name_list.size()-1].c_str(), BOT_GTK_PARAM_WIDGET_SLIDER,
            jp_it->second->joint_set[i]->limits->lower*(180/M_PI), jp_it->second->joint_set[i]->limits->upper*(180/M_PI), .01, current_dof_position*(180/M_PI));
            //bot_gtk_param_widget_add_separator (pw," ");
          }   
        } // end if         
      } // end for all joints in jp
    }// for all joint patterns
    
    
    // store current object state (will be restored on popup close).
    self->otdf_T_world_body_hold=it->second._gl_object->_T_world_body;
    self->otdf_current_jointpos_hold=it->second._gl_object->_current_jointpos;    
    
    self->dofRangeFkQueryHandler.reset();
    int num_of_increments = 5; // preallocates 5 spaces for speed
    self->dofRangeFkQueryHandler=boost::shared_ptr<BatchFKQueryHandler>(new BatchFKQueryHandler(it->second._otdf_instance,num_of_increments));

    g_signal_connect(G_OBJECT(pw), "changed", G_CALLBACK(on_otdf_dof_range_widget_changed), self);


    close_button = gtk_button_new_with_label ("Close");
    g_signal_connect (G_OBJECT (close_button),
                    "clicked",
                    G_CALLBACK (on_popup_close),
                    (gpointer) window);
    g_signal_connect(G_OBJECT(pw), "destroy",
      G_CALLBACK(on_otdf_dof_range_widget_popup_close), self); 


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
    self->instance_selection  = string(instance_name);

    if(!strcmp(name,PARAM_OTDF_ADJUST_PARAM)) {
      spawn_adjust_params_popup(self);
    }
    else if(!strcmp(name,PARAM_OTDF_ADJUST_DOF)) {
      spawn_adjust_dofs_popup(self);
    }
    else if(!strcmp(name,PARAM_OTDF_INSTANCE_CLEAR)) {
      fprintf(stderr,"\nClearing Selected Instance\n");
        
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = self->instantiated_objects.find(string(instance_name));
      
     if(it!=self->instantiated_objects.end())
     {
        self->instantiated_objects.erase(it);
        if(self->object_selection==string(instance_name))
        {
          self->link_selection = " ";
          self->object_selection = " ";
        }  

        typedef map<string, StickyHandStruc > sticky_hands_map_type_;
       
        sticky_hands_map_type_::iterator hand_it = self->sticky_hands.begin();
        while (hand_it!=self->sticky_hands.end()) {
            string hand_name = string(hand_it->second.object_name);
           if (hand_name == string(instance_name))
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
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      self->instantiated_objects.clear();
      self->sticky_hands.clear();
      for( map<string,int >::iterator it = self->instance_cnt.begin(); it!=self->instance_cnt.end(); it++)
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
      typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
      for( object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
      { 
        string instance_name = it->first;
        otdf_instance_names[i] = (char *) instance_name.c_str();
        otdf_instance_nums[i] =i;
        // cout << "Instance:  " << instance_name  << " i: "<< i << endl;
        // self->instantiated_objects_id.insert(make_pair(i,instance_name));
        ++i;
      }
    }
    else 
    {
      num_otdf_instances = 1;
      otdf_instance_names =(char **) calloc(num_otdf_instances, sizeof(char *));
      otdf_instance_nums = (int *)calloc(num_otdf_instances, sizeof(int));

      string instance_name = "No objects Instantiated";
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
