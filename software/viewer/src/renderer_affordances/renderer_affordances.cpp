#include "renderer_affordances.hpp"
#include "AffordanceCollectionListener.hpp"
#include "RobotStateListener.hpp"
#include "InitGraspOptPublisher.hpp"
#include "CandidateGraspSeedListener.hpp"
#include "GraspOptStatusListener.hpp"
#include "otdf_instance_management_gui_utils.hpp"
#include "object_interaction_gui_utils.hpp"


#define GEOM_EPSILON 1e-9

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
using namespace renderer_affordances;
using namespace renderer_affordances_gui_utils;

// =================================================================================
// DRAWING

static void _draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererAffordances *self = (RendererAffordances*) renderer;

  glEnable(GL_DEPTH_TEST);

  //-draw 
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  // glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  
  if((self->ehandler.picking)&&(self->selection_enabled)&&(self->clicked)){
        glLineWidth (3.0);
        glPushMatrix();
        glBegin(GL_LINES);
        glVertex3f(self->ray_start[0], self->ray_start[1],self->ray_start[2]); // object coord
        glVertex3f(self->ray_end[0], self->ray_end[1],self->ray_end[2]);
        glEnd();
        glPopMatrix();
        
        if(self->dragging)
        {
         Eigen::Vector3f diff = self->ray_hit_drag - self->ray_hit;
         double length =diff.norm();
         double head_width = 0.03; double head_length = 0.03;double body_width = 0.01;
         glColor4f(0,0,0,1);
         glPushMatrix();
         glTranslatef(self->ray_hit[0], self->ray_hit[1],self->ray_hit[2]);
         //--get rotation in angle/axis form
         double theta;
         Eigen::Vector3f axis;// = self->ray_hit-self->ray_start;
         axis.normalize();
         diff.normalize();
         Eigen::Vector3f uz,ux;   uz << 0 , 0 , 1;ux << 1 , 0 , 0;
         axis = ux.cross(diff);
         theta = acos(ux.dot(diff));
   
         glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); 
         glTranslatef(length/2, 0,0);
         bot_gl_draw_arrow_3d(length,head_width, head_length,body_width);
         glPopMatrix();
        }
        
  }

  float c[3] = {0.3,0.3,0.6};
  float alpha = 0.8;

  // Draw all OTDF objectes.
  typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
  for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
  {
    it->second._gl_object->enable_link_selection(self->selection_enabled);
    it->second._gl_object->draw_body(c,alpha);
  }

  // Draw all sticky hands
  float c2[3] = {0.3,0.5,0.3}; alpha = 0.6;
  typedef map<string, StickyHandStruc > sticky_hands_map_type_;
  for(sticky_hands_map_type_::const_iterator hand_it = self->sticky_hands.begin(); hand_it!=self->sticky_hands.end(); hand_it++)
  {
    hand_it->second._gl_hand->enable_link_selection(self->selection_enabled);
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(hand_it->second.object_name);
    KDL::Frame T_world_graspgeometry = KDL::Frame::Identity(); // the object might have moved.
    
    if(!obj_it->second._gl_object->get_link_frame(hand_it->second.geometry_name,T_world_graspgeometry))
       cerr << " failed to retrieve " << hand_it->second.geometry_name<<" in object " << hand_it->second.object_name <<endl;
    else {  
      double r,p,y;
      T_world_graspgeometry.M.GetRPY(r,p,y);
      hand_it->second._gl_hand->draw_body_in_frame (c2,alpha,T_world_graspgeometry);//draws in grasp_geometry frame
      }
    
  }

}
// =================================================================================
// EVENT HANDLING
// ----------------------------------------------------------------------------
static double pick_query (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3])
{
  RendererAffordances *self = (RendererAffordances*) ehandler->user;
  if(self->selection_enabled==0){
    return -1.0;
  }
  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  if(ray_start[2]<0)
   plane_pt << 0,0,5;
  else
   plane_pt << 0,0,-5;
 
   
  double lambda1 = ray_dir[0] * plane_normal[0]+
                   ray_dir[1] * plane_normal[1]+
                   ray_dir[2] * plane_normal[2];
   // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < 1e-9) return -1.0;

   double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
       (plane_pt[1] - ray_start[1]) * plane_normal[1] +
       (plane_pt[2] - ray_start[2]) * plane_normal[2];
   double t = fabs(lambda2 / lambda1);// =1;
  
  to << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
  self->ray_start = from;
  self->ray_end = to;
  self->ray_hit_t = t;
  self->ray_hit_drag = to;
  self->ray_hit = to;
  collision::Collision_Object * intersected_object = NULL;
  Eigen::Vector3f hit_pt;
  double shortest_distance = -1;
  typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
  // loop through object list and check if ray intersect any of them.
  for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
  {

    if(it->second._gl_object) // to make sure that _gl_object is initialized 
    {
      it->second._gl_object->_collision_detector->num_collisions();
      it->second._gl_object->_collision_detector->ray_test( from, to, intersected_object,hit_pt);
      // Highlight all objects that intersect with ray
      if(intersected_object != NULL ){
            self->ray_hit = hit_pt;
            self->ray_hit_t = (hit_pt - self->ray_start).norm();
            Eigen::Vector3f diff = (from-hit_pt);
            double distance = diff.norm();
            if(shortest_distance>0) {
              if (distance < shortest_distance)
                shortest_distance = distance;
            }
            else
              shortest_distance = distance;
            intersected_object = NULL;  
      }
    }
  }// end for


  //loop through stick-hands list and check if ray intersect any of them.
  typedef map<string, StickyHandStruc > sticky_hands_map_type_;
  for(sticky_hands_map_type_::iterator it = self->sticky_hands.begin(); it!=self->sticky_hands.end(); it++)
  {
  
        KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();       
        typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
        object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(it->second.object_name);
        if(!obj_it->second._gl_object->get_link_frame(it->second.geometry_name,T_world_graspgeometry))
            cerr << " failed to retrieve " << it->second.geometry_name<<" in object " << it->second.object_name <<endl;
        else {
            KDL::Frame T_graspgeometry_world = T_world_graspgeometry.Inverse();
            KDL::Vector temp,temp2;
            //convert to geometry frame.
            temp[0]=from[0];temp[1]=from[1];temp[2]=from[2];
            temp = T_graspgeometry_world*temp;         
            temp2[0]=to[0];temp2[1]=to[1];temp2[2]=to[2];           
            temp2 = T_graspgeometry_world*temp2;
            Eigen::Vector3f from_geomframe,to_geomframe; 
            from_geomframe[0]=(float) temp[0];from_geomframe[1]=(float) temp[1];from_geomframe[2]=(float) temp[2];
            to_geomframe[0]=(float) temp2[0];to_geomframe[1]=(float) temp2[1];to_geomframe[2]=(float) temp2[2];
            Eigen::Vector3f hit_pt;
            it->second._gl_hand->_collision_detector->num_collisions();
            it->second._gl_hand->_collision_detector->ray_test( from_geomframe, to_geomframe, intersected_object,hit_pt);

            if(intersected_object != NULL ){
              Eigen::Vector3f diff = (from_geomframe-hit_pt);
              double distance = diff.norm();
              if(shortest_distance>0) {
                if (distance < shortest_distance)
                  shortest_distance = distance;
              }
              else
                shortest_distance = distance;
              intersected_object = NULL;
            }
      
       }// end if
  
  }// end for
  return shortest_distance;
}

// ----------------------------------------------------------------------------
static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
  RendererAffordances *self = (RendererAffordances*) ehandler->user;
  //std::cout << "Aff ehandler->picking " << ehandler->picking << std::endl;
  if((ehandler->picking==0)||(self->selection_enabled==0)){

     (*self->object_selection)  = " ";
     (*self->link_selection)  = " ";

    //loop through object list and clear old selections.
    typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
    for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
    {
      if(it->second._gl_object) // to make sure that _gl_object is initialized 
      {
        it->second._gl_object->highlight_link((*self->link_selection));
      }
    }// end for
  

    //loop through stick-hands list and clear older Selections
    typedef map<string, StickyHandStruc > sticky_hands_map_type_;
    for(sticky_hands_map_type_::iterator it = self->sticky_hands.begin(); it!=self->sticky_hands.end(); it++)
    {
      string no_selection = " ";
      it->second._gl_hand->highlight_link(no_selection);  
    }// end for  
     
    return 0;
  } // end  if((ehandler->picking==0)||(self->selection_enabled==0))
    
  if(self->dblclk_popup){   
    fprintf(stderr, "Object DblClk Popup is Open. Closing \n");
    gtk_widget_destroy(self->dblclk_popup);
  }

  self->clicked = 1;
  //fprintf(stderr, "Mouse Press : %f,%f\n",ray_start[0], ray_start[1]);

  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  if(ray_start[2]<0)
   plane_pt << 0,0,5;
  else
   plane_pt << 0,0,-5;
 
   
  double lambda1 = ray_dir[0] * plane_normal[0]+
                   ray_dir[1] * plane_normal[1] +
                   ray_dir[2] * plane_normal[2];
   // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < 1e-9) return 0;

   double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
       (plane_pt[1] - ray_start[1]) * plane_normal[1] +
       (plane_pt[2] - ray_start[2]) * plane_normal[2];
   double t = fabs(lambda2 / lambda1);// =1;
  
  to << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
 
  self->ray_start = from;
  self->ray_end = to;
  self->ray_hit_t = t;
  self->ray_hit_drag = to;
//  cout  << "from " << from.transpose() << endl;
//  cout  << "to " << to.transpose() << endl;
  

 // KDL::Frame T_graspgeometry_handinitpos = KDL::Frame::Identity();

  collision::Collision_Object * intersected_object = NULL;
  Eigen::Vector3f hit_pt;
  typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
  // loop through object list and check if ray intersect any of them.
  for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
  {

    if(it->second._gl_object) // to make sure that _gl_object is initialized 
    {
      it->second._gl_object->_collision_detector->num_collisions();
      it->second._gl_object->_collision_detector->ray_test( from, to, intersected_object,hit_pt);

      // Highlight all objects that intersect with ray
      if(intersected_object != NULL ){
        self->ray_hit = hit_pt;
        self->ray_hit_drag = hit_pt;
        self->ray_hit_t = (hit_pt - self->ray_start).norm();
        cout << "prev selection :" << (*self->link_selection)  <<  endl;
        cout << "intersected :" << intersected_object->id().c_str() << " at: "<< hit_pt.transpose() << endl;
        cout << "object_type :" << it->second._otdf_instance->name_<<  endl;
        cout << "object_name :" << it->first<<  endl;
        (*self->object_selection)  =  it->first;
        (*self->link_selection)  = string(intersected_object->id().c_str());        
        it->second._gl_object->highlight_link((*self->link_selection)); 
        intersected_object = NULL;
      }
      else {
      // clear previous selections
       string no_selection = " ";
       it->second._gl_object->highlight_link(no_selection); 
      }

    }
  }// end for
  

  //loop through stick-hands list and check if ray intersect any of them.
  typedef map<string, StickyHandStruc > sticky_hands_map_type_;
  for(sticky_hands_map_type_::iterator it = self->sticky_hands.begin(); it!=self->sticky_hands.end(); it++)
  {
  
        KDL::Frame T_world_graspgeometry = KDL::Frame::Identity();       
        typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
        object_instance_map_type_::iterator obj_it = self->instantiated_objects.find(it->second.object_name);
        if(!obj_it->second._gl_object->get_link_frame(it->second.geometry_name,T_world_graspgeometry))
            cerr << " failed to retrieve " << it->second.geometry_name<<" in object " << it->second.object_name <<endl;
        else {
            KDL::Frame T_graspgeometry_world = T_world_graspgeometry.Inverse();
            KDL::Vector temp,temp2;
            //convert to geometry frame.
            temp[0]=from[0];temp[1]=from[1];temp[2]=from[2];
            temp = T_graspgeometry_world*temp;         
            temp2[0]=to[0];temp2[1]=to[1];temp2[2]=to[2];           
            temp2 = T_graspgeometry_world*temp2;
            Eigen::Vector3f from_geomframe,to_geomframe; 
            from_geomframe[0]=(float) temp[0];from_geomframe[1]=(float) temp[1];from_geomframe[2]=(float) temp[2];
            to_geomframe[0]=(float) temp2[0];to_geomframe[1]=(float) temp2[1];to_geomframe[2]=(float) temp2[2];
            
            it->second._gl_hand->_collision_detector->num_collisions();
            it->second._gl_hand->_collision_detector->ray_test( from_geomframe, to_geomframe, intersected_object);

            // Highlight all objects that intersect with ray
            if(intersected_object != NULL ){
              cout << "prev selection :" << (*self->link_selection)  <<  endl;
              cout << "intersected sticky hand:" << intersected_object->id().c_str()  << endl;
              cout << "object_name :" << it->second.object_name<<  endl;
              cout << "geom_name :" << it->second.geometry_name<<  endl;
              cout << "stickyhand_name :" << it->first<<  endl;
             // (*self->object_selection)  =  it->first;
             // (*self->link_selection)  = string(intersected_object->id().c_str());
              it->second._gl_hand->enable_whole_body_selection(true); 
              string sticky_hand_name = string(intersected_object->id().c_str());       
              it->second._gl_hand->highlight_link(sticky_hand_name); 
              intersected_object = NULL;
            }
            else {
              string no_selection = " ";
              it->second._gl_hand->highlight_link(no_selection);
            }

       }// end if
  
  }// end for
  
 //(event->button==3) -- Right Click
  //cout << "current selection:" << (*self->link_selection)  <<  endl;
  if(((*self->link_selection)  != " ")&&(event->button==1)&&(event->type==GDK_2BUTTON_PRESS)){
    //spawn_object_geometry_dblclk_popup(self);
    // draw circle for angle specification around the axis.
    self->dragging = 1;
    self->show_popup_onrelease = 1;
    bot_viewer_request_redraw(self->viewer);
    std::cout << "RendererAffordances: Event is consumed" <<  std::endl;
    return 1;// consumed if pop up comes up.
  }

  bot_viewer_request_redraw(self->viewer);
  return 0; // not consumed if pop up does not come up.
  
}


// ----------------------------------------------------------------------------
static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3],  const GdkEventButton *event)
{
  RendererAffordances *self = (RendererAffordances*) ehandler->user;
  self->clicked = 0;

  if((ehandler->picking==0)||(self->selection_enabled==0)){
    return 0;
  }  
  if(self->show_popup_onrelease){
      spawn_object_geometry_dblclk_popup(self); // DblClk POPUP!
      self->show_popup_onrelease = 0;
  } 
  if (self->dragging) {
    self->dragging = 0;
  }
  if (ehandler->picking==1)
    ehandler->picking=0; //if picking release picking (Important)
  bot_viewer_request_redraw(self->viewer);
  return 1;
}

// ----------------------------------------------------------------------------
static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,  const double ray_start[3], const double ray_dir[3],   const GdkEventMotion *event)
{
  RendererAffordances *self = (RendererAffordances*) ehandler->user;
  
  if((!self->dragging)||(ehandler->picking==0)||(self->selection_enabled==0)){
    return 0;
  }
  if(self->show_popup_onrelease){
    double t = self->ray_hit_t;
    self->ray_hit_drag << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
    std::cout << "motion!!! set sticky hand orientation"  <<  std::endl;
  }
  bot_viewer_request_redraw(self->viewer);
  return 1;
}

// =================================================================================
// WIDGET MANAGEMENT AND RENDERER CONSTRUCTION

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererAffordances *self = (RendererAffordances*) user;
  if(!strcmp(name, PARAM_MANAGE_INSTANCES)) {
    fprintf(stderr,"\nClicked Manage Instances\n");
    spawn_instance_management_popup(self);

  }
  else if (! strcmp (name, PARAM_OTDF_SELECT)) {
   self->otdf_id = bot_gtk_param_widget_get_enum (self->pw, PARAM_OTDF_SELECT);
  }
  else if(!strcmp(name, PARAM_INSTANTIATE)) {
    cout << "\nInstantiating Selected Otdf:  " << self->otdf_filenames[self->otdf_id] << endl;
    create_otdf_object_instance(self);
  }
  else if(!strcmp(name, PARAM_CLEAR)) {
    fprintf(stderr,"\nClearing Instantiated Objects\n");
    
     self->instantiated_objects.clear();
     self->sticky_hands.clear();
     for( map<string,int >::iterator it = self->instance_cnt.begin(); it!=self->instance_cnt.end(); it++)
     { 
       it->second = 0;
     }
     bot_viewer_request_redraw(self->viewer);
  }
  else if (! strcmp(name, PARAM_SELECTION)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_SELECTION)) {
      //bot_viewer_request_pick (self->viewer, &(self->ehandler));
      self->selection_enabled = 1;
    }
    else{
      self->selection_enabled = 0;
    }
  }
  else if (! strcmp (name, PARAM_LHAND_URDF_SELECT)) {
   self->lhand_urdf_id = bot_gtk_param_widget_get_enum (self->pw, PARAM_LHAND_URDF_SELECT);
  }
  else if (! strcmp (name, PARAM_RHAND_URDF_SELECT)) {
   self->rhand_urdf_id = bot_gtk_param_widget_get_enum (self->pw, PARAM_RHAND_URDF_SELECT);
  } 

  
}

static void
_free (BotRenderer *renderer)
{
  free (renderer);
}

BotRenderer *renderer_affordances_new (BotViewer *viewer, int render_priority, lcm_t *lcm)
{

  RendererAffordances *self = (RendererAffordances*) calloc (1, sizeof (RendererAffordances));
  self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
  self->robot_name_ptr  = new string(" ");
  self->affordanceMsgHandler = boost::shared_ptr<AffordanceCollectionListener>(new AffordanceCollectionListener(self));
  self->robotStateListener = boost::shared_ptr<RobotStateListener>(new RobotStateListener(self));
  self->candidateGraspSeedListener = boost::shared_ptr<CandidateGraspSeedListener>(new CandidateGraspSeedListener(self));
  self->initGraspOptPublisher  = boost::shared_ptr<InitGraspOptPublisher>(new InitGraspOptPublisher(self));
  self->graspOptStatusListener= boost::shared_ptr<GraspOptStatusListener>(new GraspOptStatusListener(self));
  self->free_running_sticky_hand_cnt = 0;
  self->T_graspgeometry_handinitpos= KDL::Frame::Identity(); 
  
  self->viewer = viewer;
  self->renderer.draw = _draw;
  self->renderer.destroy = _free;
  self->renderer.name = (char*) RENDERER_NAME;
  self->renderer.user = self;
  self->renderer.enabled = 1;

  BotEventHandler *ehandler = &self->ehandler;
  ehandler->name = (char*) RENDERER_NAME;
  ehandler->enabled = 1;
  ehandler->pick_query = pick_query;
  ehandler->key_press = NULL;
  ehandler->hover_query = NULL;
  ehandler->mouse_press = mouse_press;
  ehandler->mouse_release = mouse_release;
  ehandler->mouse_motion = mouse_motion;
  ehandler->user = self;

  string otdf_models_path = string(getModelsPath()) + "/otdf/"; // getModelsPath gives /drc/software/build/models/
  self->otdf_dir_name_ptr = new string(otdf_models_path);
  cout << "searching for otdf files in: "<< (*self->otdf_dir_name_ptr) << endl;
  vector<string> otdf_files = vector<string>();
  get_OTDF_filenames_from_dir(otdf_models_path.c_str(),otdf_files);
  cout << "found " << otdf_files.size() << " files"<< endl;
  self->num_otdfs = otdf_files.size();
  self->otdf_names =(char **) calloc(self->num_otdfs, sizeof(char *));
  self->otdf_nums = (int *)calloc(self->num_otdfs, sizeof(int));
  self->instance_cnt.clear();
  self->instantiated_objects.clear();
  self->sticky_hands.clear();
  self->instance_selection_ptr = new string(" ");
  
  for(size_t i=0;i<otdf_files.size();i++){
   cout << otdf_files[i] << endl;
   self->otdf_filenames.push_back(otdf_files[i]);
   self->instance_cnt.insert(make_pair(otdf_files[i], (int)0));
   self->otdf_names[i]=(char *) otdf_files[i].c_str();
   self->otdf_nums[i] =i;
  }

  string urdf_models_path = string(getModelsPath()) + "/mit_gazebo_models/mit_robot_hands/"; // getModelsPath gives /drc/software/build/models/
  self->urdf_dir_name_ptr = new string(urdf_models_path);
  cout << "searching for hand urdf files in: "<< (*self->urdf_dir_name_ptr) << endl;
  vector<string> urdf_files = vector<string>();
  get_URDF_or_SDF_filenames_from_dir(urdf_models_path.c_str(),urdf_files);
  cout << "found " << urdf_files.size() << " files"<< endl;
  self->num_urdfs = urdf_files.size();
  self->urdf_names =(char **) calloc(self->num_urdfs, sizeof(char *));
  self->urdf_nums = (int *)calloc(self->num_urdfs, sizeof(int));   
  
  for(size_t i=0;i<urdf_files.size();i++){
   cout << urdf_files[i] << endl;
   self->urdf_filenames.push_back(urdf_files[i]);
   self->urdf_names[i]=(char *) urdf_files[i].c_str();
   self->urdf_nums[i] =i;
  }


  bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

  self->otdf_id= 0; // default file
  self->lhand_urdf_id= 1; // default file
  self->rhand_urdf_id= 2; // default file
  bot_gtk_param_widget_add_separator (self->pw,"Objects");
  bot_gtk_param_widget_add_enumv (self->pw, PARAM_OTDF_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                          self->otdf_id,
				                          self->num_otdfs,
			                            (const char **)  self->otdf_names,
			                            self->otdf_nums);

  bot_gtk_param_widget_add_buttons(self->pw,PARAM_INSTANTIATE, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_MANAGE_INSTANCES, NULL);
  bot_gtk_param_widget_add_buttons(self->pw,PARAM_CLEAR, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SELECTION, 0, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_OPT_POOL_READY, 0, NULL);
  bot_gtk_param_widget_add_separator (self->pw,"Sticky Hands");
  bot_gtk_param_widget_add_enumv (self->pw, PARAM_LHAND_URDF_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                          self->lhand_urdf_id,
				                          self->num_urdfs,
			                            (const char **)  self->urdf_names,
			                            self->urdf_nums);
  bot_gtk_param_widget_add_enumv (self->pw, PARAM_RHAND_URDF_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                          self->rhand_urdf_id,
				                          self->num_urdfs,
			                            (const char **)  self->urdf_names,
			                            self->urdf_nums); 
  
  
  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  self->renderer.widget = GTK_WIDGET(self->pw);


	self->selection_enabled = 1;
	bot_gtk_param_widget_set_bool(self->pw, PARAM_SELECTION,self->selection_enabled);
	bool optpoolready = self->graspOptStatusListener->isOptPoolReady();
	bot_gtk_param_widget_set_bool(self->pw,PARAM_OPT_POOL_READY,optpoolready);
  self->clicked = 0;	
  self->dragging = 0;
  self->show_popup_onrelease = 0;
	self->link_selection = new string(" ");
  self->object_selection = new string(" ");

  return &self->renderer;
}

void setup_renderer_affordances(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
  bot_viewer_add_renderer_on_side(viewer, renderer_affordances_new(viewer, render_priority, lcm), render_priority, 0); // 0= add on left hand side
}
