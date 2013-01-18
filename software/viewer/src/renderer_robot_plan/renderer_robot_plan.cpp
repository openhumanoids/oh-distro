#include <iostream>
#include <boost/function.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "lcmtypes/drc_lcmtypes.h"

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
#define PARAM_HIDE "Hide Plan"  
#define PARAM_PLAN_PART "Part of Plan"  
#define DRAW_PERSIST_SEC 4

#define PARAM_NEW_VICON_PLAN "Get Vicon Plan"

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
  
  // Our only source of a free running clock:
  int64_t robot_utime;
  
  // Vicon seed planning collection settings:
  int vicon_n_plan_samples;
  double vicon_sample_period;
  int8_t vicon_type;
  
  
} RendererRobotPlan;

static void
_renderer_free (BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  free(self);
}


//=================================


// Convert number to jet colour coordinates
// In: number between 0-->1
// Out: rgb jet colours 0->1
// http://metastine.com/2011/01/implementing-a-continuous-jet-colormap-function-in-glsl/
static inline void jet_rgb(float value,float rgb[]){
  float fourValue = (float) 4 * value;
  rgb[0]   = std::min(fourValue - 1.5, -fourValue + 4.5);
  rgb[1] = std::min(fourValue - 0.5, -fourValue + 3.5);
  rgb[2]  = std::min(fourValue + 0.5, -fourValue + 2.5);
  for (int i=0;i<3;i++){
   if (rgb[i] <0) {
     rgb[i] =0;
   }else if (rgb[i] >1){
     rgb[i] =1;
   }
  }
}


static void 
draw_state(BotViewer *viewer, BotRenderer *super, uint i){
  float c[3] = {0.3,0.3,0.6}; // light blue
  float alpha = 0.4;
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  // Each model Jet: blue to red
  float j = (float)i/ (self->robotPlanListener->_gl_robot_list.size() -1);
  jet_rgb(j,c);
  glColor4f(c[0],c[1],c[2], alpha);
  
  self->robotPlanListener->_gl_robot_list[i]->show_bbox(self->visualize_bbox);
  self->robotPlanListener->_gl_robot_list[i]->enable_link_selection(self->picking);
  //if((*self->selection)!=" ")
  self->robotPlanListener->_gl_robot_list[i]->highlight_link((*self->selection));
  self->robotPlanListener->_gl_robot_list[i]->draw_body (c,alpha);  
}

static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super)
{
  RendererRobotPlan *self = (RendererRobotPlan*) super->user;
  // if hide is enabled - then dont draw the plan:
  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_HIDE)) {
     return;
  }
  
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
  
  
  int plan_size =   self->robotPlanListener->_gl_robot_list.size();
  if (plan_size ==0) // nothing to renderer
    return;
  
  
  
  //glColor3f(c[0],c[1],c[2]);
  
  double plan_part = bot_gtk_param_widget_get_double(self->pw, PARAM_PLAN_PART);
  
  if (plan_part==1){
    //printf("Show it all\n");
  
    for(uint i = 0; i < self->robotPlanListener->_gl_robot_list.size(); i++) 
    { 
      draw_state(viewer,super,i);
    }
  }else{
    uint w_plan = (uint) round(plan_part* (plan_size -1));
    //printf("                                  Show around %f of %d    %d\n", plan_part, plan_size, w_plan);
    draw_state(viewer,super,w_plan);
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
  collision::Collision_Object * intersected_object = NULL;
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


static void onRobotUtime (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_utime_t *msg, void *user){
  RendererRobotPlan *self = (RendererRobotPlan*) user;
  self->robot_utime = msg->utime;
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
  else if(! strcmp(name, PARAM_NEW_VICON_PLAN)) {
    drc::plan_collect_t msg;
    msg.utime = self->robot_utime;//bot_timestamp_now();
    msg.type = self->vicon_type;
    msg.n_plan_samples = self->vicon_n_plan_samples;
    msg.sample_period = self->vicon_sample_period;
    self->lcm->publish("VICON_GET_PLAN", &msg);
    bot_viewer_set_status_bar_message(self->viewer, "Sent VICON_GET_PLAN [nsamples: %d, period %fsec] @ %lld",msg.n_plan_samples, msg.sample_period, msg.utime);    
    
  }

  bot_viewer_request_redraw(self->viewer);
  
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
    
    // default Vicon plan sample values:
    self->vicon_n_plan_samples = 20;
    self->vicon_sample_period = 0.5;

    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);
    
    // C-style subscribe:
    drc_utime_t_subscribe(self->lcm->getUnderlyingLCM(),"ROBOT_UTIME",onRobotUtime,self); 

    
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_PICKING, 0, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_WIRE, 0, NULL);

    bot_gtk_param_widget_add_buttons(self->pw, PARAM_NEW_VICON_PLAN, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_HIDE, 0, NULL);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_PLAN_PART,
                                   BOT_GTK_PARAM_WIDGET_SLIDER,
                                   0, 1, 0.01, 1);    
    
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

