#include "renderer_affordances.hpp"
#include "AffordanceCollectionListener.hpp"
#include "RobotStateListener.hpp"
#include "otdf_instance_management_gui_utils.hpp"
#include "object_interaction_gui_utils.hpp"


////////////////// THE FOLLOWING CODE WAS COPIED IN HERE TO AVOID
////////////////// DEPENDENCY WITH THE COMMON_UTILS/GEOM_UTILS POD [MFALLON]
#define GEOM_EPSILON 1e-9

using namespace std;
using namespace boost;
using namespace visualization_utils;
using namespace collision;
using namespace renderer_affordances;
using namespace renderer_affordances_gui_utils;

namespace renderer_affordances{


// this function should go into otdf_utils library
int get_OTDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
      std::string fn =string(dirp->d_name);
      if(fn.substr(fn.find_last_of(".") + 1) == "otdf") 
        files.push_back(fn.substr(0,fn.find_last_of(".")));
    }
    closedir(dp);
    return 0;
}

}//end naemspace

////////////////////////////// END OF CODE COPIED IN FROM COMMON_UTILS


static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererAffordances *self = (RendererAffordances*) renderer;

  glEnable(GL_DEPTH_TEST);

  //-draw 
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  // glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  
  if((self->picking)&&(self->clicked)){
        glLineWidth (3.0);
        glPushMatrix();
        glBegin(GL_LINES);
        glVertex3f(self->ray_start[0], self->ray_start[1],self->ray_start[2]); // object coord
        glVertex3f(self->ray_end[0], self->ray_end[1],self->ray_end[2]);
        glEnd();
        glPopMatrix();
  }

  float c[3] = {0.3,0.3,0.6};
  float alpha = 0.8;
  //glColor3f(c[0],c[1],c[2]);
  glColor4f(c[0],c[1],c[2],alpha);

  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
  {
    it->second._gl_object->enable_link_selection(self->picking);
    it->second._gl_object->draw_body(c,alpha);
  }

}


static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererAffordances *self = (RendererAffordances*) ehandler->user;
  if(self->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  self->clicked = 1;
  fprintf(stderr, "Mouse Press : %f,%f\n",ray_start[0], ray_start[1]);

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
  std::cout  << "from " << from.transpose() << std::endl;
  std::cout  << "to " << to.transpose() << std::endl;
  
  
  collision::Collision_Object * intersected_object = NULL;
  bool selection_made = false;
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  for(object_instance_map_type_::const_iterator it = self->instantiated_objects.begin(); it!=self->instantiated_objects.end(); it++)
  {

    if(it->second._gl_object) // to make sure that _gl_object is initialized 
    {
      it->second._gl_object->_collision_detector->num_collisions();
      it->second._gl_object->_collision_detector->ray_test( from, to, intersected_object );

      // Highlight all objects that intersect with ray
      if(intersected_object != NULL ){
        std::cout << "prev selection :" << (*self->link_selection)  <<  std::endl;
        std::cout << "intersected :" << intersected_object->id().c_str() <<  std::endl;
        std::cout << "object_type :" << it->second._otdf_instance->name_<<  std::endl;
        std::cout << "object_name :" << it->first<<  std::endl;
        (*self->object_selection)  =  it->first;
        (*self->link_selection)  = std::string(intersected_object->id().c_str());        
        it->second._gl_object->highlight_link((*self->link_selection)); 
        intersected_object = NULL;
        if(!selection_made)
          selection_made = true;
      }
      else{
      std::string no_selection = " ";
        it->second._gl_object->highlight_link(no_selection);  //Clear older Selections
      }
    }
  }// end for
  
  if (!selection_made){ // if no object intersected clear old selections
        (*self->object_selection)  = " ";
        (*self->link_selection)  = " ";
  }
 
std::cout << "current selection:" << (*self->link_selection)  <<  std::endl;
if(((*self->link_selection)  != " ")&&(event->button==3)&&(event->type==GDK_2BUTTON_PRESS)){
  spawn_object_geometry_rtclk_popup(self);
}
  


  bot_viewer_request_redraw(self->viewer);
  return 1;
}



static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventButton *event)
{
  RendererAffordances *self = (RendererAffordances*) ehandler->user;
  self->clicked = 0;
  if(self->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  
  bot_viewer_request_redraw(self->viewer);

  return 0;
}

// =================================================================================


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
    std::cout << "\nInstantiating Selected Otdf:  " << self->otdf_filenames[self->otdf_id] << std::endl;
    create_otdf_object_instance(self);
    bot_viewer_request_pick (self->viewer, &(self->ehandler));
  }
  else if(!strcmp(name, PARAM_CLEAR)) {
    fprintf(stderr,"\nClearing Instantiated Objects\n");
    
     self->instantiated_objects.clear();
     for( std::map<std::string,int >::iterator it = self->instance_cnt.begin(); it!=self->instance_cnt.end(); it++)
     { 
       it->second = 0;
     }
     bot_viewer_request_redraw(self->viewer);
  }
  else if (! strcmp(name, PARAM_PICKING)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_PICKING)) {
      bot_viewer_request_pick (self->viewer, &(self->ehandler));
      self->picking = 1;
    }
    else{
      self->picking = 0;
    }
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

  self->affordanceMsgHandler = boost::shared_ptr<AffordanceCollectionListener>(new AffordanceCollectionListener(self));
  self->robotStateListener = boost::shared_ptr<RobotStateListener>(new RobotStateListener(self->lcm,viewer));

  self->viewer = viewer;
  self->renderer.draw = _draw;
  self->renderer.destroy = _free;
  self->renderer.name = (char*) RENDERER_NAME;
  self->renderer.user = self;
  self->renderer.enabled = 1;

  BotEventHandler *ehandler = &self->ehandler;
  ehandler->name = (char*) RENDERER_NAME;
  ehandler->enabled = 1;
  ehandler->pick_query = NULL;
  ehandler->key_press = NULL;
  ehandler->hover_query = NULL;
  ehandler->mouse_press = mouse_press;
  ehandler->mouse_release = mouse_release;
  ehandler->mouse_motion = NULL;
  ehandler->user = self;

  string otdf_models_path = std::string(getModelsPath()) + "/otdf/"; // getModelsPath gives /drc/software/build/models/
  self->otdf_dir_name_ptr = new std::string(otdf_models_path);
  std::cout << "searching for otdf files in: "<< (*self->otdf_dir_name_ptr) << std::endl;
  std::vector<std::string> otdf_files = std::vector<std::string>();
  get_OTDF_filenames_from_dir(otdf_models_path.c_str(),otdf_files);
  std::cout << "found " << otdf_files.size() << " files"<< std::endl;
  self->num_otdfs = otdf_files.size();
  self->otdf_names =(char **) calloc(self->num_otdfs, sizeof(char *));
  self->otdf_nums = (int *)calloc(self->num_otdfs, sizeof(int));
  self->instance_cnt.clear();
  self->instantiated_objects.clear();
  self->instance_selection_ptr = new std::string(" ");

  for(size_t i=0;i<otdf_files.size();i++){
   std::cout << otdf_files[i] << std::endl;
   self->otdf_filenames.push_back(otdf_files[i]);
   self->instance_cnt.insert(std::make_pair(otdf_files[i], (int)0));
   self->otdf_names[i]=(char *) otdf_files[i].c_str();
   self->otdf_nums[i] =i;
  }

  bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

  self->otdf_id= 0; // default file

  bot_gtk_param_widget_add_enumv (self->pw, PARAM_OTDF_SELECT, BOT_GTK_PARAM_WIDGET_MENU, 
				                          self->otdf_id,
				                          self->num_otdfs,
			                            (const char **)  self->otdf_names,
			                            self->otdf_nums);

  bot_gtk_param_widget_add_buttons(self->pw,PARAM_INSTANTIATE, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_MANAGE_INSTANCES, NULL);
  bot_gtk_param_widget_add_buttons(self->pw,PARAM_CLEAR, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_PICKING, 0, NULL);
  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  self->renderer.widget = GTK_WIDGET(self->pw);


	self->picking = 0;
  self->clicked = 0;	
	self->link_selection = new std::string(" ");
  self->object_selection = new std::string(" ");
  return &self->renderer;
}

void setup_renderer_affordances(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
  bot_viewer_add_renderer_on_side(viewer, renderer_affordances_new(viewer, render_priority, lcm), render_priority, 0); // 0= add on left hand side
}
