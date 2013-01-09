#include <iostream>
#include <boost/function.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <GL/gl.h>
#include <bot_vis/bot_vis.h>
#include <bot_core/rotations.h>
#include <gdk/gdkkeysyms.h>
#include <Eigen/Dense>

#include "renderer_robot_plan.hpp"
#include "RobotPlanListener.hpp"


#define RENDERER_NAME "Robot Plan Display"
#define PARAM_PICKING "Enable Selection"
#define PARAM_WIRE "Show BBoxs For Meshes"  
#define DRAW_PERSIST_SEC 4

using namespace std;
using namespace boost;
using namespace renderer_robot_plan;

typedef struct _RendererRobotPlan
{
  BotRenderer renderer;
  BotViewer          *viewer;
  BotGtkParamWidget *pw;
  boost::shared_ptr<RobotPlanListener> robotPlanListener;
  boost::shared_ptr<lcm::LCM> lcm;
  int64_t max_draw_utime;
  BotEventHandler ehandler;
  bool picking;
  bool clicked;
  bool visualize_bbox;
  Eigen::Vector3f ray_start;
  Eigen::Vector3f ray_end;
  std::string* selection;
} RendererRobotPlan;

static void
_renderer_free (BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  free(self);
}


//=================================


static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  int64_t now = bot_timestamp_now();
  self->max_draw_utime = self->robotPlanListener->_last_plan_msg_timestamp  + DRAW_PERSIST_SEC * 1000000;	
  if(now > self->max_draw_utime)
    return; // clear robot plan display
  
  glEnable(GL_DEPTH_TEST);

  //-draw 
  //glPointSize(5.0f);
  //glBegin(GL_POINTS);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
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
  
 double c[3] = {0.3,0.3,0.6};
 double alpha = 0.2;
  //glColor3f(c[0],c[1],c[2]);
  glColor4f(c[0],c[1],c[2],alpha);
  
  for(uint i = 0; i < self->robotPlanListener->_gl_robot_list.size(); i++) 
  { 
    self->robotPlanListener->_gl_robot_list[i]->show_bbox(self->visualize_bbox);
    self->robotPlanListener->_gl_robot_list[i]->enable_link_selection(self->picking);
    //if((*self->selection)!=" ")
      self->robotPlanListener->_gl_robot_list[i]->highlight_link((*self->selection));
    self->robotPlanListener->_gl_robot_list[i]->draw_body (c,alpha);
  }

}


static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;
  if(self->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  self->clicked = 1;
//  if(event->button==3)
//   fprintf(stderr, "Right Click (event->button): %d\n",event->button);
  fprintf(stderr, "Mouse Press : %f,%f\n",ray_start[0], ray_start[1]);
  
  

  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  plane_pt << 0,0,0;
  double lambda1 = ray_dir[0] * plane_normal[0]+
                   ray_dir[1] * plane_normal[1] +
                   ray_dir[2] * plane_normal[2];
   // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < 1e-9) return 0;

   double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
       (plane_pt[1] - ray_start[1]) * plane_normal[1] +
       (plane_pt[2] - ray_start[2]) * plane_normal[2];
   double t = lambda2 / lambda1;// =1;
  
  to << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
 
    self->ray_start = from;
  self->ray_end = to;
  std::cout  << "from " << from.transpose() << std::endl;
  std::cout  << "to " << to.transpose() << std::endl;
  
   //to << ray_dir[0], ray_dir[1], ray_dir[2];
 //std::cout  << "num_coll_objects: " << self->robotStateListener->_collision_object_map.size() <<  std::endl;
 //std::cout  << "num_colls: " << self->robotStateListener->_collision_detector->num_collisions() <<  std::endl;// segfaults
  collision_detection::Collision_Object * intersected_object = NULL;
  for(uint i = 0; i < self->robotPlanListener->_gl_robot_list.size(); i++) 
  { 
    if(self->robotPlanListener->_gl_robot_list[i]) // to make sure that _gl_robot is initialized 
    {
     self->robotPlanListener->_gl_robot_list[i]->_collision_detector->num_collisions();
     self->robotPlanListener->_gl_robot_list[i]->_collision_detector->ray_test( from, to, intersected_object );
    }
    if( intersected_object != NULL ){
      std::cout << "prev selection :" << (*self->selection)  <<  std::endl;
      std::cout << "intersected :" << intersected_object->id().c_str() <<  std::endl;
      (*self->selection)  = std::string(intersected_object->id().c_str());
      self->robotPlanListener->_gl_robot_list[i]->highlight_link((*self->selection));
    }
   else
   (*self->selection)  = " ";
  }

  bot_viewer_request_redraw(self->viewer);

  return 1;
}

static int 
mouse_release (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererRobotPlan *self = (RendererRobotPlan*) ehandler->user;
  self->clicked = 0;
  if(self->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  
  bot_viewer_request_redraw(self->viewer);
  return 1;
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererRobotPlan *self = (RendererRobotPlan*) user;
  if (! strcmp(name, PARAM_PICKING)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_PICKING)) {
      bot_viewer_request_pick (self->viewer, &(self->ehandler));
      self->picking = 1;
    }
    else{
      self->picking = 0;
    }
  }
  else if(! strcmp(name, PARAM_WIRE)) {
    if (bot_gtk_param_widget_get_bool(pw, PARAM_WIRE)){
      self->visualize_bbox= true;  
    }
    else{
      self->visualize_bbox = false;
    }
  }
}
void 
setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererRobotPlan *self = (RendererRobotPlan*) calloc (1, sizeof (RendererRobotPlan));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
    self->robotPlanListener = boost::shared_ptr<RobotPlanListener>(new RobotPlanListener(self->lcm, 
												    viewer));

    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = (char *) RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;

    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);
    
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_PICKING, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_WIRE, 0, NULL);
      
  	g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  	self->picking = 0;
    self->clicked = 0;	
  	self->selection = new std::string(" ");
    self->visualize_bbox = false;

    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
    
        
    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = NULL;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);
    
}

